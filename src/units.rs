use alloc::vec::Vec;

pub use uom::si;
pub use uom::si::f64::{
    Acceleration, AngularVelocity, ElectricCurrent, ElectricPotential, Length, Time, Velocity,
};

use crate::Headers;

#[allow(unreachable_pub, unused_imports)]
pub(crate) mod prelude {
    pub use super::si::acceleration::{meter_per_second_squared as mps2, standard_gravity};
    pub use super::si::angular_velocity::degree_per_second;
    pub use super::si::electric_current::{ampere, milliampere};
    pub use super::si::electric_potential::{millivolt, volt};
    pub use super::si::length::meter;
    pub use super::si::time::{microsecond, second};
    pub use super::si::velocity::meter_per_second;
    pub use super::{
        Acceleration, AngularVelocity, ElectricCurrent, ElectricPotential, Length, Time, Velocity,
    };
}

include_generated!("failsafe_phase");
include_generated!("flight_mode");
include_generated!("state");

pub(crate) mod new {
    use super::*;

    pub(crate) fn time(raw: u64) -> Time {
        Time::new::<prelude::microsecond>(raw as f64)
    }

    pub(crate) fn acceleration(raw: i32, headers: &Headers) -> Acceleration {
        let gs = f64::from(raw) / f64::from(headers.acceleration_1g.unwrap());
        Acceleration::new::<prelude::standard_gravity>(gs)
    }

    pub(crate) fn angular_velocity(raw: i32, headers: &Headers) -> AngularVelocity {
        let scale = headers.gyro_scale.unwrap();
        let rad = f64::from(scale) * f64::from(raw);

        AngularVelocity::new::<si::angular_velocity::radian_per_second>(rad)
    }

    pub(crate) fn current(raw: i32) -> ElectricCurrent {
        // Correct from BF 3.1.7 (3.1.0?), INAV 2.0.0
        ElectricCurrent::new::<si::electric_current::centiampere>(raw.into())
    }

    pub(crate) fn vbat(raw: u32) -> ElectricPotential {
        // Correct from BF 4.0.0, INAV 3.0.0?
        ElectricPotential::new::<si::electric_potential::centivolt>(raw.into())
    }

    pub(crate) fn velocity(raw: u32) -> Velocity {
        Velocity::new::<si::velocity::centimeter_per_second>(raw.into())
    }
}

pub trait FlagSet {
    type Flag: Flag;

    /// Checks if a given flag is enabled.
    fn is_set(&self, flag: Self::Flag) -> bool;

    /// Returns the names of all enabled flags.
    fn as_names(&self) -> Vec<&'static str>;
}

pub trait Flag {
    /// Returns the name of this flag.
    fn as_name(&self) -> &'static str;
}

#[cfg(test)]
mod tests {
    use super::*;

    // Audit #8: Betaflight 2025.12+ (4.6) inserted ALTHOLD@4, CHIRP@6, POSHOLD@9 into the
    // flight-mode bitmap, shifting everything from bit 4 up. The bug was Betaflight2025
    // reusing the 4.5 table. Verify the same raw bit now decodes differently per firmware.
    #[test]
    fn flight_mode_post_4_5_bit_ordering() {
        use crate::headers::InternalFirmware;
        use alloc::vec;
        let names = |raw: u32, fw: InternalFirmware| FlightModeSet::new(raw, fw).as_names();

        // 2025 (POST_4_5) ordering.
        assert_eq!(names(1 << 4, InternalFirmware::Betaflight2025), vec!["ALTHOLD"]);
        assert_eq!(names(1 << 6, InternalFirmware::Betaflight2025), vec!["CHIRP"]);
        assert_eq!(names(1 << 9, InternalFirmware::Betaflight2025), vec!["POSHOLD"]);
        assert_eq!(names(1 << 10, InternalFirmware::Betaflight2025), vec!["GPS RESCUE"]);
        assert_eq!(names(1 << 11, InternalFirmware::Betaflight2025), vec!["ANTI GRAVITY"]);

        // 4.5 keeps the older ordering at those same bits.
        assert_eq!(names(1 << 4, InternalFirmware::Betaflight4_5), vec!["HEADFREE"]);
        assert_eq!(names(1 << 7, InternalFirmware::Betaflight4_5), vec!["GPS RESCUE"]);
        assert_eq!(names(1 << 8, InternalFirmware::Betaflight4_5), vec!["ANTI GRAVITY"]);

        // Shared low bits (0..=3) decode identically across both.
        assert_eq!(names(1 << 0, InternalFirmware::Betaflight2025), vec!["ARM"]);
        assert_eq!(names(1 << 3, InternalFirmware::Betaflight4_5), vec!["MAG"]);
    }

    macro_rules! float_eq {
        ($left:expr, $right:expr) => {
            let epsilon = 0.0001;
            let diff = ($left - $right).abs();
            assert!(
                diff < epsilon,
                "{left} and {right} are greater than {epsilon} apart: {diff}",
                left = $left,
                right = $right
            );
        };
    }

    #[test]
    fn electric_current() {
        float_eq!(1.39, new::current(139).get::<prelude::ampere>());
    }

    #[test]
    fn electric_potential() {
        float_eq!(16.32, new::vbat(1632).get::<prelude::volt>());
    }

    mod resolution {
        use super::*;

        #[test]
        fn time() {
            use si::time::{day, microsecond};

            let ms = Time::new::<microsecond>(1.);
            float_eq!(1., ms.get::<microsecond>());

            let d = Time::new::<day>(1.);
            float_eq!(1., d.get::<day>());

            float_eq!(
                ms.get::<microsecond>() + d.get::<microsecond>(),
                (ms + d).get::<microsecond>()
            );
        }

        #[test]
        fn acceleration() {
            use si::acceleration::{millimeter_per_second_squared as mmps2, standard_gravity};

            let milli_gs = Acceleration::new::<standard_gravity>(0.001);
            float_eq!(0.001, milli_gs.get::<standard_gravity>());

            let hecto_gs = Acceleration::new::<standard_gravity>(100.);
            float_eq!(100., hecto_gs.get::<standard_gravity>());

            float_eq!(
                milli_gs.get::<mmps2>() + hecto_gs.get::<mmps2>(),
                (milli_gs + hecto_gs).get::<mmps2>()
            );
        }

        #[test]
        fn angular_velocity() {
            use si::angular_velocity::degree_per_second as dps;

            let slow = AngularVelocity::new::<dps>(0.01);
            float_eq!(0.01, slow.get::<dps>());

            let fast = AngularVelocity::new::<dps>(5_000.);
            float_eq!(5_000., fast.get::<dps>());

            float_eq!(5_000.01, (slow + fast).get::<dps>());
        }

        #[test]
        fn electric_current() {
            use si::electric_current::{kiloampere, milliampere};

            let ma = ElectricCurrent::new::<milliampere>(1.);
            float_eq!(1., ma.get::<milliampere>());

            let ka = ElectricCurrent::new::<kiloampere>(1.);
            float_eq!(1., ka.get::<kiloampere>());

            float_eq!(
                ma.get::<milliampere>() + ka.get::<milliampere>(),
                (ma + ka).get::<milliampere>()
            );
        }

        #[test]
        fn electric_potential() {
            use si::electric_potential::{kilovolt, millivolt};

            let mv = ElectricPotential::new::<millivolt>(1.);
            float_eq!(1., mv.get::<millivolt>());

            let kv = ElectricPotential::new::<kilovolt>(1.);
            float_eq!(1., kv.get::<kilovolt>());

            float_eq!(
                mv.get::<millivolt>() + kv.get::<millivolt>(),
                (mv + kv).get::<millivolt>()
            );
        }
    }
}
