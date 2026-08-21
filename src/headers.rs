//! Types for the header section of blackbox logs.

use alloc::borrow::ToOwned as _;
use alloc::string::String;
use core::str::FromStr;
use core::{cmp, fmt, str};

use hashbrown::HashMap;
use time::PrimitiveDateTime;

use crate::frame::gps::{GpsFrameDef, GpsFrameDefBuilder};
use crate::frame::gps_home::{GpsHomeFrameDef, GpsHomeFrameDefBuilder};
use crate::frame::main::{MainFrameDef, MainFrameDefBuilder};
use crate::frame::slow::{SlowFrameDef, SlowFrameDefBuilder};
use crate::frame::{is_frame_def_header, parse_frame_def_header, DataFrameKind};
use crate::parser::{InternalError, InternalResult};
use crate::predictor::Predictor;
use crate::{DataParser, FilterSet, Reader, Unit};

include_generated!("debug_mode");
include_generated!("disabled_fields");
include_generated!("features");
include_generated!("pwm_protocol");

pub type ParseResult<T> = Result<T, ParseError>;

/// A fatal error encountered while parsing the headers of a log.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "_serde", derive(serde::Serialize))]
pub enum ParseError {
    /// The log uses a data format version that is unsupported or could not be
    /// parsed.
    UnsupportedDataVersion,
    /// The `Firmware revision` header could not be parsed, or is from an
    /// unsupported firmware.
    InvalidFirmware(String),
    /// The log comes from an unsupported version of a known firmware.
    UnsupportedFirmwareVersion(Firmware),
    /// Could not parse the value in header `header`.
    InvalidHeader { header: String, value: String },
    // NOTE: Consider including header name in this variant for better error messages
    /// Did not find a required header.
    MissingHeader,
    /// The file ended before the start of the data section.
    IncompleteHeaders,
    /// Definition for frame type `frame` is missing required a required field.
    MissingField { frame: DataFrameKind, field: String },
    /// Unknown unrecoverable error in the frame definition.
    MalformedFrameDef(DataFrameKind),
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedDataVersion => write!(f, "unsupported or invalid data version"),
            Self::InvalidFirmware(firmware) => write!(f, "could not parse firmware: `{firmware}`"),
            Self::UnsupportedFirmwareVersion(firmware) => {
                let name = firmware.name();
                let version = firmware.version();
                write!(f, "logs from {name} v{version} are not supported")
            }
            Self::InvalidHeader { header, value } => {
                write!(f, "invalid value for header `{header}`: `{value}`")
            }
            Self::MissingHeader => {
                write!(f, "one or more headers required for parsing are missing")
            }
            Self::IncompleteHeaders => write!(f, "end of file found before data section"),
            Self::MissingField { frame, field } => {
                write!(f, "missing field `{field}` in `{frame}` frame definition")
            }
            Self::MalformedFrameDef(frame) => write!(f, "malformed {frame} frame definition"),
        }
    }
}

impl core::error::Error for ParseError {}

/// Decoded headers containing metadata for a blackbox log.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct Headers<'data> {
    data: Reader<'data>,

    main_frame_def: MainFrameDef<'data>,
    slow_frame_def: SlowFrameDef<'data>,
    gps_frame_def: Option<GpsFrameDef<'data>>,
    gps_home_frame_def: Option<GpsHomeFrameDef<'data>>,

    firmware_revision: &'data str,
    pub(crate) internal_firmware: InternalFirmware,
    firmware: Firmware,
    firmware_date: Option<&'data str>,
    board_info: Option<&'data str>,
    craft_name: Option<&'data str>,

    debug_mode: DebugMode,
    disabled_fields: DisabledFields,
    features: FeatureSet,
    pwm_protocol: PwmProtocol,

    /// The battery voltage measured at arm.
    pub(crate) vbat_reference: Option<u16>,
    /// Calibration for the accelerometer.
    pub(crate) acceleration_1g: Option<u16>,
    /// Calibration for the gyro in radians / second.
    pub(crate) gyro_scale: Option<f32>,

    pub(crate) min_throttle: Option<u16>,
    pub(crate) motor_output_range: Option<MotorOutputRange>,

    /// `I interval`: how many loop iterations lie between intra (keyframe) frames.
    pub(crate) frame_interval_i: u32,
    /// `P interval` numerator/denominator: the fraction of loop iterations that get
    /// an inter frame written. Firmware below 1/1 deliberately omits frames.
    pub(crate) frame_interval_p_num: u32,
    pub(crate) frame_interval_p_denom: u32,

    unknown: HashMap<&'data str, &'data str>,
}

impl<'data> Headers<'data> {
    /// Parses only the headers of a blackbox log.
    ///
    /// `data` will be advanced to the start of the data section of the log,
    /// ready to be passed to [`DataParser::new`][crate::DataParser::new].
    ///
    /// **Note:** This assumes that `data` is aligned to the start of a log.
    pub(crate) fn parse(data: &'data [u8]) -> ParseResult<Self> {
        let mut data = Reader::new(data);

        // Skip product header
        let product = data.read_line();
        debug_assert_eq!(crate::MARKER.strip_suffix(b"\n"), product);
        let data_version = data.read_line();
        if !matches!(data_version, Some(b"H Data version:2")) {
            return Err(ParseError::UnsupportedDataVersion);
        }

        let mut state = State::new();

        loop {
            if data.peek() != Some(b'H') {
                break;
            }

            let restore = data.get_restore_point();
            let (name, value) = match parse_header(&mut data) {
                Ok(x) => x,
                Err(InternalError::Retry) => {
                    tracing::debug!("found corrupted header");
                    data.restore(restore);
                    break;
                }
                Err(InternalError::Eof) => return Err(ParseError::IncompleteHeaders),
            };

            if !state.update(name, value) {
                return Err(ParseError::InvalidHeader {
                    header: name.to_owned(),
                    value: value.to_owned(),
                });
            }
        }

        state.finish(data)
    }

    fn validate(&self) -> ParseResult<()> {
        let has_accel = self.acceleration_1g.is_some();
        let has_min_throttle = self.min_throttle.is_some();
        // NOTE: Could additionally verify motor_0 is in main frame definition
        let motor_0 = self.main_frame_def.index_motor_0;
        let has_vbat_ref = self.vbat_reference.is_some();
        let has_min_motor = self.motor_output_range.is_some();
        let has_gps_home = self.gps_home_frame_def.is_some();

        let predictor = |frame, field, predictor, index| {
            let ok = match predictor {
                Predictor::MinThrottle => has_min_throttle,
                Predictor::Motor0 => motor_0.is_some() && index > motor_0.unwrap(),
                Predictor::HomeLat | Predictor::HomeLon => has_gps_home,
                Predictor::VBatReference => has_vbat_ref,
                Predictor::MinMotor => has_min_motor,
                Predictor::Zero
                | Predictor::Previous
                | Predictor::StraightLine
                | Predictor::Average2
                | Predictor::Increment
                | Predictor::FifteenHundred
                | Predictor::LastMainFrameTime => true,
            };

            if ok {
                Ok(())
            } else {
                tracing::error!(field, ?predictor, "bad predictor");
                Err(ParseError::MalformedFrameDef(frame))
            }
        };

        let unit = |frame, field, unit| {
            if unit == Unit::Acceleration && !has_accel {
                tracing::error!(field, ?unit, "bad unit");
                Err(ParseError::MalformedFrameDef(frame))
            } else {
                Ok(())
            }
        };

        self.main_frame_def.validate(predictor, unit)?;
        self.slow_frame_def.validate(predictor, unit)?;

        if let Some(ref def) = self.gps_frame_def {
            def.validate(predictor, unit)?;
        }

        if let Some(ref def) = self.gps_home_frame_def {
            def.validate(predictor, unit)?;
        }

        Ok(())
    }
}

impl<'data> Headers<'data> {
    /// Returns a new [`DataParser`] without beginning parsing.
    pub fn data_parser<'headers>(&'headers self) -> DataParser<'data, 'headers> {
        DataParser::new(self.data.clone(), self, &FilterSet::default())
    }

    pub fn data_parser_with_filters<'headers>(
        &'headers self,
        filters: &FilterSet,
    ) -> DataParser<'data, 'headers> {
        DataParser::new(self.data.clone(), self, filters)
    }
}

/// Getters for various log headers.
impl<'data> Headers<'data> {
    #[inline]
    pub fn main_frame_def(&self) -> &MainFrameDef<'data> {
        &self.main_frame_def
    }

    #[inline]
    pub fn slow_frame_def(&self) -> &SlowFrameDef<'data> {
        &self.slow_frame_def
    }

    #[inline]
    pub fn gps_frame_def(&self) -> Option<&GpsFrameDef<'data>> {
        self.gps_frame_def.as_ref()
    }

    #[inline]
    pub(crate) fn gps_home_frame_def(&self) -> Option<&GpsHomeFrameDef<'data>> {
        self.gps_home_frame_def.as_ref()
    }

    /// The full `Firmware revision` header.
    ///
    /// Consider using the [`firmware`][Self::firmware] method instead.
    #[inline]
    pub fn firmware_revision(&self) -> &'data str {
        self.firmware_revision
    }

    /// The firmware that wrote the log.
    #[inline]
    pub fn firmware(&self) -> Firmware {
        self.firmware
    }

    /// The `Firmware date` header
    pub fn firmware_date(&self) -> Option<Result<PrimitiveDateTime, &'data str>> {
        let format = time::macros::format_description!(
            "[month repr:short case_sensitive:false] [day padding:space] [year] [hour \
             repr:24]:[minute]:[second]"
        );
        self.firmware_date
            .map(|date| PrimitiveDateTime::parse(date, &format).map_err(|_| date))
    }

    /// The `Board info` header.
    #[inline]
    pub fn board_info(&self) -> Option<&'data str> {
        self.board_info
    }

    /// The `Craft name` header.
    #[inline]
    pub fn craft_name(&self) -> Option<&'data str> {
        self.craft_name
    }

    #[inline]
    pub fn debug_mode(&self) -> DebugMode {
        self.debug_mode
    }

    #[inline]
    pub fn disabled_fields(&self) -> DisabledFields {
        self.disabled_fields
    }

    #[inline]
    pub fn features(&self) -> FeatureSet {
        self.features
    }

    #[inline]
    pub fn pwm_protocol(&self) -> PwmProtocol {
        self.pwm_protocol
    }

    /// The configured `minthrottle`, if logged. For analog ESC protocols this is
    /// the bottom of the motor output range used to scale motor values to percent.
    #[inline]
    pub fn min_throttle(&self) -> Option<u16> {
        self.min_throttle
    }

    /// The logged `motorOutput` range as `(min, max)`, if present.
    #[inline]
    pub fn motor_output_range(&self) -> Option<(u16, u16)> {
        self.motor_output_range.map(|r| (r.min, r.max))
    }

    /// Any unknown headers.
    #[inline]
    pub fn unknown(&self) -> &HashMap<&'data str, &'data str> {
        &self.unknown
    }

    /// Whether the firmware would have written a frame for loop iteration `index`.
    ///
    /// With `blackbox_sample_rate` below 1/1 the firmware deliberately omits some
    /// inter frames; this reproduces its decision so the decoder can tell an
    /// intentional gap from a dropped frame. Mirrors BBLV's `shouldHaveFrame`.
    pub(crate) const fn should_have_frame(&self, index: u32) -> bool {
        (index % self.frame_interval_i + self.frame_interval_p_num - 1)
            % self.frame_interval_p_denom
            < self.frame_interval_p_num
    }

    /// Counts the frames the firmware intentionally skipped between `last_iteration`
    /// and the next frame it actually wrote.
    ///
    /// `loopIteration` uses the `Increment` predictor, so without this the decoded
    /// iteration counter drifts away from the flight controller's on any log with
    /// `blackbox_sample_rate` below 1/1. Mirrors BBLV's
    /// `countIntentionallySkippedFrames`.
    pub(crate) fn count_intentionally_skipped_frames(&self, last_iteration: u32) -> u32 {
        // A run of skipped frames can never be longer than the P denominator, so this
        // both bounds the loop and guards against a nonsensical header combination.
        let mut index = last_iteration.wrapping_add(1);
        let mut skipped = 0;
        while skipped < self.frame_interval_p_denom && !self.should_have_frame(index) {
            skipped += 1;
            index = index.wrapping_add(1);
        }
        skipped
    }
}

/// A supported firmware.
///
/// This is not the same as the `Firmware type` header since all modern
/// firmwares set that to `Cleanflight`. This is instead decoded from `Firmware
/// revision`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "_serde", derive(serde::Serialize))]
pub enum Firmware {
    /// [Betaflight](https://github.com/betaflight/betaflight/)
    Betaflight(FirmwareVersion),
    /// [INAV](https://github.com/iNavFlight/inav/)
    Inav(FirmwareVersion),
}

impl Firmware {
    pub const fn name(&self) -> &'static str {
        match self {
            Firmware::Betaflight(_) => "Betaflight",
            Firmware::Inav(_) => "INAV",
        }
    }

    pub const fn version(&self) -> FirmwareVersion {
        let (Self::Betaflight(version) | Self::Inav(version)) = self;
        *version
    }

    fn parse(firmware_revision: &str) -> Result<Self, ParseError> {
        let invalid_fw = || Err(ParseError::InvalidFirmware(firmware_revision.to_owned()));

        let mut iter = firmware_revision.split(' ');

        let kind = iter.next().map(str::to_ascii_lowercase);
        let Some(version) = iter.next().and_then(FirmwareVersion::parse) else {
            return invalid_fw();
        };

        let (fw, is_supported) = match kind.as_deref() {
            Some("betaflight") => (
                Firmware::Betaflight(version),
                crate::BETAFLIGHT_SUPPORT.contains(&version),
            ),
            Some("inav") => (
                Firmware::Inav(version),
                crate::INAV_SUPPORT.contains(&version),
            ),
            Some("emuflight") => {
                tracing::error!("EmuFlight is not supported");
                return invalid_fw();
            }
            _ => {
                tracing::error!("Could not parse firmware revision");
                return invalid_fw();
            }
        };

        if is_supported {
            Ok(fw)
        } else {
            Err(ParseError::UnsupportedFirmwareVersion(fw))
        }
    }
}

impl PartialOrd for Firmware {
    fn partial_cmp(&self, other: &Self) -> Option<cmp::Ordering> {
        match (self, other) {
            (Firmware::Betaflight(fw_self), Firmware::Betaflight(fw_other))
            | (Firmware::Inav(fw_self), Firmware::Inav(fw_other)) => fw_self.partial_cmp(fw_other),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FirmwareVersion {
    pub major: u16,
    pub minor: u16,
    pub patch: u16,
}

impl FirmwareVersion {
    pub const fn new(major: u16, minor: u16, patch: u16) -> Self {
        Self {
            major,
            minor,
            patch,
        }
    }

    fn parse(s: &str) -> Option<Self> {
        let mut components = s.splitn(3, '.').map(|s| s.parse().ok());

        let major = components.next()??;
        let minor = components.next()??;
        let patch = components.next()??;

        Some(Self {
            major,
            minor,
            patch,
        })
    }
}

impl fmt::Display for FirmwareVersion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}.{}", self.major, self.minor, self.patch)
    }
}

#[cfg(feature = "_serde")]
impl serde::Serialize for FirmwareVersion {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use alloc::string::ToString as _;
        serializer.serialize_str(&self.to_string())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum InternalFirmware {
    Betaflight4_2,
    Betaflight4_3,
    Betaflight4_4,
    Betaflight4_5,
    Betaflight2025,
    Betaflight2026,
    Inav5,
    Inav6,
    Inav7,
    Inav8,
    Inav9,
}

impl InternalFirmware {
    pub(crate) const fn is_betaflight(self) -> bool {
        match self {
            Self::Betaflight4_2
            | Self::Betaflight4_3
            | Self::Betaflight4_4
            | Self::Betaflight4_5
            | Self::Betaflight2025
            | Self::Betaflight2026 => true,
            Self::Inav5 | Self::Inav6 | Self::Inav7 | Self::Inav8 | Self::Inav9 => false,
        }
    }

    #[expect(unused)]
    pub(crate) const fn is_inav(self) -> bool {
        // Will need to be changed if any new firmwares are added
        !self.is_betaflight()
    }
}

impl From<Firmware> for InternalFirmware {
    fn from(fw: Firmware) -> Self {
        #[expect(clippy::wildcard_enum_match_arm)]
        match fw {
            Firmware::Betaflight(FirmwareVersion {
                major: 4, minor: 2, ..
            }) => Self::Betaflight4_2,
            Firmware::Betaflight(FirmwareVersion {
                major: 4, minor: 3, ..
            }) => Self::Betaflight4_3,
            Firmware::Betaflight(FirmwareVersion {
                major: 4, minor: 4, ..
            }) => Self::Betaflight4_4,
            Firmware::Betaflight(FirmwareVersion {
                major: 4, minor: 5, ..
            }) => Self::Betaflight4_5,
            Firmware::Betaflight(FirmwareVersion { major: 2025, .. }) => Self::Betaflight2025,
            Firmware::Betaflight(FirmwareVersion { major: 2026.., .. }) => Self::Betaflight2026,
            Firmware::Inav(FirmwareVersion { major: 5, .. }) => Self::Inav5,
            Firmware::Inav(FirmwareVersion { major: 6, .. }) => Self::Inav6,
            Firmware::Inav(FirmwareVersion { major: 7, .. }) => Self::Inav7,
            Firmware::Inav(FirmwareVersion { major: 8, .. }) => Self::Inav8,
            Firmware::Inav(FirmwareVersion { major: 9, .. }) => Self::Inav9,
            _ => unreachable!(),
        }
    }
}

impl PartialOrd for InternalFirmware {
    fn partial_cmp(&self, other: &Self) -> Option<cmp::Ordering> {
        if self.is_betaflight() != other.is_betaflight() {
            return None;
        }

        Some((*self as u8).cmp(&(*other as u8)))
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct MotorOutputRange {
    pub(crate) min: u16,
    pub(crate) max: u16,
}

impl MotorOutputRange {
    pub(crate) fn from_str(s: &str) -> Option<Self> {
        let (min, max) = s.split_once(',')?;
        let min = min.parse().ok()?;
        let max = max.parse().ok()?;
        Some(Self { min, max })
    }
}

#[derive(Debug)]
struct RawHeaderValue<'data, T> {
    header: &'data str,
    raw: &'data str,
    value: T,
}

impl<T> RawHeaderValue<'_, T> {
    fn invalid_header_error(&self) -> ParseError {
        ParseError::InvalidHeader {
            header: self.header.to_owned(),
            value: self.raw.to_owned(),
        }
    }
}

impl<'data, T: FromStr> RawHeaderValue<'data, T> {
    fn parse(header: &'data str, raw: &'data str) -> Result<Self, <T as FromStr>::Err> {
        Ok(Self {
            header,
            raw,
            value: raw.parse()?,
        })
    }
}

#[derive(Debug)]
struct State<'data> {
    main_frames: MainFrameDefBuilder<'data>,
    slow_frames: SlowFrameDefBuilder<'data>,
    gps_frames: GpsFrameDefBuilder<'data>,
    gps_home_frames: GpsHomeFrameDefBuilder<'data>,

    firmware_revision: Option<&'data str>,
    firmware_date: Option<&'data str>,
    firmware_kind: Option<&'data str>,
    board_info: Option<&'data str>,
    craft_name: Option<&'data str>,

    debug_mode: Option<RawHeaderValue<'data, u32>>,
    disabled_fields: u32,
    features: u32,
    pwm_protocol: Option<RawHeaderValue<'data, u32>>,

    vbat_reference: Option<u16>,
    acceleration_1g: Option<u16>,
    gyro_scale: Option<f32>,

    min_throttle: Option<u16>,
    motor_output_range: Option<MotorOutputRange>,

    unknown: HashMap<&'data str, &'data str>,
}

impl<'data> State<'data> {
    fn new() -> Self {
        Self {
            main_frames: MainFrameDef::builder(),
            slow_frames: SlowFrameDef::builder(),
            gps_frames: GpsFrameDef::builder(),
            gps_home_frames: GpsHomeFrameDef::builder(),

            firmware_revision: None,
            firmware_date: None,
            firmware_kind: None,
            board_info: None,
            craft_name: None,

            debug_mode: None,
            disabled_fields: 0,
            features: 0,
            pwm_protocol: None,

            vbat_reference: None,
            acceleration_1g: None,
            gyro_scale: None,

            min_throttle: None,
            motor_output_range: None,

            unknown: HashMap::new(),
        }
    }

    /// Returns `true` if the header/value pair was valid
    fn update(&mut self, header: &'data str, value: &'data str) -> bool {
        // Using closure + Result for early return pattern (Rust doesn't have try blocks yet)
        (|| -> Result<(), ()> {
            match header {
                "Firmware revision" => self.firmware_revision = Some(value),
                "Firmware date" => self.firmware_date = Some(value),
                "Firmware type" => self.firmware_kind = Some(value),
                "Board information" => self.board_info = Some(value),
                "Craft name" => self.craft_name = Some(value),

                "debug_mode" => {
                    let debug_mode = RawHeaderValue::parse(header, value).map_err(|_| ())?;
                    self.debug_mode = Some(debug_mode);
                }
                "fields_disabled_mask" => self.disabled_fields = value.parse().map_err(|_| ())?,
                "features" => self.features = value.parse::<i32>().map_err(|_| ())?.cast_unsigned(),
                "motor_pwm_protocol" => {
                    let protocol = RawHeaderValue::parse(header, value).map_err(|_| ())?;
                    self.pwm_protocol = Some(protocol);
                }

                "vbatref" => {
                    let vbat_reference = value.parse().map_err(|_| ())?;
                    self.vbat_reference = Some(vbat_reference);
                }
                "acc_1G" => {
                    let one_g = value.parse().map_err(|_| ())?;
                    self.acceleration_1g = Some(one_g);
                }
                "gyro.scale" | "gyro_scale" => {
                    let scale = if let Some(hex) = value.strip_prefix("0x") {
                        u32::from_str_radix(hex, 16).map_err(|_| ())?
                    } else {
                        value.parse().map_err(|_| ())?
                    };

                    let scale = f32::from_bits(scale);
                    self.gyro_scale = Some(scale.to_radians());
                }
                "minthrottle" => {
                    let min_throttle = value.parse().map_err(|_| ())?;
                    self.min_throttle = Some(min_throttle);
                }
                "motorOutput" => {
                    let range = MotorOutputRange::from_str(value).ok_or(())?;
                    self.motor_output_range = Some(range);
                }

                _ if is_frame_def_header(header) => {
                    let (frame_kind, property) = parse_frame_def_header(header).unwrap();

                    match frame_kind {
                        DataFrameKind::Inter | DataFrameKind::Intra => {
                            self.main_frames.update(frame_kind, property, value);
                        }
                        DataFrameKind::Slow => self.slow_frames.update(property, value),
                        DataFrameKind::Gps => self.gps_frames.update(property, value),
                        DataFrameKind::GpsHome => self.gps_home_frames.update(property, value),
                    }
                }

                // Legacy calibration headers
                "vbatscale" | "vbat_scale" | "currentMeter" | "currentSensor" => {}

                header => {
                    tracing::debug!("skipping unknown header: `{header}` = `{value}`");
                    self.unknown.insert(header, value);
                }
            }

            Ok(())
        })()
        .is_ok()
    }

    fn finish(self, data: Reader<'data>) -> ParseResult<Headers<'data>> {
        let not_empty = |s: &&str| !s.is_empty();

        let firmware_revision = self.firmware_revision.ok_or(ParseError::MissingHeader)?;
        let firmware = Firmware::parse(firmware_revision)?;
        let internal_firmware = firmware.into();

        let (frame_interval_i, frame_interval_p_num, frame_interval_p_denom) =
            parse_frame_intervals(&self.unknown);

        // NOTE: Error source location could be improved with tracing/spans
        let headers = Headers {
            data,

            main_frame_def: self.main_frames.parse()?,
            slow_frame_def: self.slow_frames.parse()?,
            gps_frame_def: self.gps_frames.parse()?,
            gps_home_frame_def: self.gps_home_frames.parse()?,

            firmware_revision,
            internal_firmware,
            firmware,
            firmware_date: self.firmware_date,
            board_info: self.board_info.map(str::trim).filter(not_empty),
            craft_name: self.craft_name.map(str::trim).filter(not_empty),

            // An unrecognised debug mode must not cost the user the whole log. New
            // firmware adds debug modes faster than this table tracks them, and the
            // mode only labels the `debug[]` channels -- every other field still
            // decodes correctly. Degrade to `None` (channels shown unlabelled) rather
            // than failing the parse, which is what a hard error here used to do.
            debug_mode: self.debug_mode.map_or(DebugMode::None, |raw| {
                DebugMode::new(raw.value, internal_firmware).unwrap_or_else(|| {
                    tracing::warn!(
                        "unknown debug_mode `{}` for {internal_firmware:?}",
                        raw.raw
                    );
                    DebugMode::None
                })
            }),
            disabled_fields: DisabledFields::new(self.disabled_fields, internal_firmware),
            features: FeatureSet::new(self.features, internal_firmware),
            pwm_protocol: self
                .pwm_protocol
                .ok_or(ParseError::MissingHeader)
                .and_then(|raw| {
                    PwmProtocol::new(raw.value, internal_firmware)
                        .ok_or_else(|| raw.invalid_header_error())
                })?,

            vbat_reference: self.vbat_reference,
            acceleration_1g: self.acceleration_1g,
            gyro_scale: self.gyro_scale,

            min_throttle: self.min_throttle,
            motor_output_range: self.motor_output_range,

            frame_interval_i,
            frame_interval_p_num,
            frame_interval_p_denom,

            unknown: self.unknown,
        };

        headers.validate()?;

        Ok(headers)
    }
}

/// Parses the `I interval` and `P interval` headers into the blackbox frame ratio.
///
/// `P interval` is written either as `num/denom` (older firmware) or as a bare
/// denominator -- current Betaflight writes `blackboxPInterval` directly. Defaults
/// mirror BBLV's `flightlog_parser.js`: I = 32, P = 1/1. All three are clamped to at
/// least 1 so the modular arithmetic in `should_have_frame` cannot divide by zero.
fn parse_frame_intervals(unknown: &HashMap<&str, &str>) -> (u32, u32, u32) {
    let i = unknown
        .get("I interval")
        .and_then(|v| v.trim().parse::<u32>().ok())
        .unwrap_or(32)
        .max(1);

    let (num, denom) = unknown.get("P interval").map_or((1, 1), |raw| {
        let raw = raw.trim();
        raw.split_once('/').map_or_else(
            || (1, raw.parse::<u32>().unwrap_or(1)),
            |(n, d)| {
                (
                    n.trim().parse::<u32>().unwrap_or(1),
                    d.trim().parse::<u32>().unwrap_or(1),
                )
            },
        )
    });

    (i, num.max(1), denom.max(1))
}

/// Expects the next character to be the leading H
fn parse_header<'data>(bytes: &mut Reader<'data>) -> InternalResult<(&'data str, &'data str)> {
    match bytes.read_u8() {
        Some(b'H') => {}
        Some(_) => return Err(InternalError::Retry),
        None => return Err(InternalError::Eof),
    }

    let line = bytes.read_line().ok_or(InternalError::Eof)?;

    let line = str::from_utf8(line).map_err(|_| InternalError::Retry)?;
    let line = line.strip_prefix(' ').unwrap_or(line);
    let (name, value) = line.split_once(':').ok_or(InternalError::Retry)?;

    tracing::trace!("read header `{name}` = `{value}`");

    Ok((name, value))
}

#[cfg(test)]
mod tests {
    use super::*;

    // Audit F-10: `RawMainFrame::parse` hardcoded `skipped = 0`, so `loopIteration`
    // drifted from the flight controller's counter on any log written with
    // `blackbox_sample_rate` below 1/1. Mirrors BBLV's countIntentionallySkippedFrames.
    #[test]
    fn frame_interval_parsing() {
        let intervals = |i: &str, p: &str| {
            let mut m = HashMap::new();
            m.insert("I interval", i);
            m.insert("P interval", p);
            parse_frame_intervals(&m)
        };

        // Current firmware writes a bare `blackboxPInterval`...
        assert_eq!(intervals("32", "4"), (32, 1, 4));
        // ...older firmware writes num/denom.
        assert_eq!(intervals("32", "1/2"), (32, 1, 2));
        // Degenerate values are clamped rather than dividing by zero.
        assert_eq!(intervals("0", "0"), (1, 1, 1));
        assert_eq!(parse_frame_intervals(&HashMap::new()), (32, 1, 1));
    }

    #[test]
    fn counts_intentionally_skipped_frames() {
        // Mirrors Headers::count_intentionally_skipped_frames over a bare interval
        // triple, so the arithmetic can be checked without building a full Headers.
        let count = |i: u32, num: u32, denom: u32, last: u32| {
            let should = |index: u32| (index % i + num - 1) % denom < num;
            let mut index = last.wrapping_add(1);
            let mut skipped = 0;
            while skipped < denom && !should(index) {
                skipped += 1;
                index = index.wrapping_add(1);
            }
            skipped
        };

        // 1/1 logs every iteration: nothing is ever skipped.
        for last in 0..8 {
            assert_eq!(count(32, 1, 1, last), 0, "1/1 skipped at {last}");
        }

        // 1/4 writes iterations 0, 4, 8 ... so three are skipped between each.
        assert_eq!(count(32, 1, 4, 0), 3);
        assert_eq!(count(32, 1, 4, 4), 3);

        // 1/2 skips one between each.
        assert_eq!(count(32, 1, 2, 0), 1);
        assert_eq!(count(32, 1, 2, 2), 1);
    }

    #[test]
    #[should_panic(expected = "Retry")]
    fn invalid_utf8() {
        let mut b = Reader::new(b"H \xFF:\xFF\n");
        parse_header(&mut b).unwrap();
    }
}
