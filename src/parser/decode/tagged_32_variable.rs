use super::sign_extend;
use crate::parser::{InternalError, InternalResult};
use crate::Reader;

const COUNT: usize = 3;

/// Decodes TAG2_3SVARIABLE encoding (encoding value 10).
///
/// Similar to TAG2_3S32 but with different bit widths for cases 1 and 2:
/// - Case 0 (tag 0b00): 2-2-2 bits per field (1 byte total)
/// - Case 1 (tag 0b01): 5-5-4 bits per field (2 bytes total)
/// - Case 2 (tag 0b10): 8-7-7 bits per field (3 bytes total)
/// - Case 3 (tag 0b11): variable 8/16/24/32 bits per field (tag-selected)
pub(crate) fn tagged_32_variable(data: &mut Reader) -> InternalResult<[i32; COUNT]> {
    fn read_u8_or_eof(bytes: &mut Reader) -> InternalResult<u8> {
        bytes.read_u8().ok_or(InternalError::Eof)
    }

    let mut result = [0; COUNT];

    let byte = read_u8_or_eof(data)?;
    match (byte & 0xC0) >> 6 {
        // 2 bits per field: ss11 2233
        0 => {
            result[0] = sign_extend::<2>(((byte >> 4) & 0x03).into());
            result[1] = sign_extend::<2>(((byte >> 2) & 0x03).into());
            result[2] = sign_extend::<2>((byte & 0x03).into());
        }

        // 5-5-4 bits per field: ss11 1112 2222 3333
        1 => {
            let byte2 = read_u8_or_eof(data)?;

            result[0] = sign_extend::<5>((((byte & 0x3E) >> 1) as u32) & 0x1F);
            result[1] = sign_extend::<5>(
                ((u32::from(byte & 0x01) << 4) | (u32::from(byte2 & 0xF0) >> 4)) & 0x1F,
            );
            result[2] = sign_extend::<4>(u32::from(byte2 & 0x0F));
        }

        // 8-7-7 bits per field: ss11 1111 1122 2222 2333 3333
        2 => {
            let byte2 = read_u8_or_eof(data)?;
            let byte3 = read_u8_or_eof(data)?;

            result[0] = sign_extend::<8>(
                (u32::from(byte & 0x3F) << 2) | (u32::from(byte2 & 0xC0) >> 6),
            );
            result[1] = sign_extend::<7>(
                (u32::from(byte2 & 0x3F) << 1) | (u32::from(byte3 & 0x80) >> 7),
            );
            result[2] = sign_extend::<7>(u32::from(byte3 & 0x7F));
        }

        // Variable 8/16/24/32 bits per field
        3.. => {
            let mut tags = byte & 0x3F;
            for x in &mut result {
                let tag = tags & 3;
                tags >>= 2;

                *x = match tag {
                    // 8 bits
                    0 => read_u8_or_eof(data)?.cast_signed().into(),

                    // 16 bits
                    1 => data
                        .read_u16()
                        .ok_or(InternalError::Eof)?
                        .cast_signed()
                        .into(),

                    // 24 bits
                    2 => {
                        let x = data.read_u24().ok_or(InternalError::Eof)?;
                        sign_extend::<24>(x)
                    }

                    // 32 bits
                    3.. => data.read_u32().ok_or(InternalError::Eof)?.cast_signed(),
                }
            }
        }
    }

    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn case0_two_bits() {
        // Tag 0b00, values: 0, -1, 1 → bits: 00 11 01 → 0x0D
        let b = [0x0D];
        let mut b = Reader::new(&b);

        assert_eq!([0, -1, 1], tagged_32_variable(&mut b).unwrap());
        assert!(b.is_empty());
    }

    #[test]
    fn case1_five_five_four_bits() {
        // Tag 0b01:
        // Byte 1: 01|AAAAA  where A=00001 (value 1), lower bit goes to byte2
        // Byte 2: BBBBB|CCCC where B=00010 (value 2), C=0011 (value 3)
        //
        // field0 = 5 bits = 00001 = 1
        // Layout: ss|AAAA|B  -> byte1 = 01_00010_0 = 0x44
        //         BBBB|CCCC  -> byte2 = 0010_0011 = 0x23
        //
        // field0 = bits 5..1 of byte1 = (0x44 & 0x3E) >> 1 = (0x04) >> 1 = 0x02 = 2
        // field1 = bit 0 of byte1 << 4 | bits 7..4 of byte2
        //        = (0x44 & 0x01) << 4 | (0x23 >> 4) = 0 | 2 = 2
        // field2 = bits 3..0 of byte2 = 0x23 & 0x0F = 3
        let b = [0x44, 0x23];
        let mut b = Reader::new(&b);

        assert_eq!([2, 2, 3], tagged_32_variable(&mut b).unwrap());
        assert!(b.is_empty());
    }

    #[test]
    fn case2_eight_seven_seven_bits() {
        // Tag 0b10:
        // field0 = 8 bits (6 from byte1, 2 from byte2)
        // field1 = 7 bits (6 from byte2, 1 from byte3)
        // field2 = 7 bits (7 from byte3)
        //
        // field0 = 1 = 0b00000001
        //   byte1 lower 6: 000000
        //   byte2 upper 2: 01
        // field1 = 2 = 0b0000010
        //   byte2 lower 6: 000001
        //   byte3 upper 1: 0
        // field2 = 3 = 0b0000011
        //   byte3 lower 7: 0000011
        //
        // byte1 = 10_000000 = 0x80
        // byte2 = 01_000001 = 0x41
        // byte3 = 0_0000011 = 0x03
        let b = [0x80, 0x41, 0x03];
        let mut b = Reader::new(&b);

        assert_eq!([1, 2, 3], tagged_32_variable(&mut b).unwrap());
        assert!(b.is_empty());
    }

    #[test]
    fn case3_eight_bit() {
        // Tag 0b11, all 8-bit (selector 0b00 for each)
        // tags byte: 11_00_00_00 = 0xC0
        let b = [0xC0, 0x01, 0x02, 0x03];
        let mut b = Reader::new(&b);

        assert_eq!([1, 2, 3], tagged_32_variable(&mut b).unwrap());
        assert!(b.is_empty());
    }

    #[test]
    fn case3_sixteen_bit() {
        // Tag 0b11, all 16-bit (selector 0b01 for each)
        // tags byte: 11_01_01_01 = 0xD5
        let b = [0xD5, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00];
        let mut b = Reader::new(&b);

        assert_eq!([1, 2, 3], tagged_32_variable(&mut b).unwrap());
        assert!(b.is_empty());
    }

    #[test]
    fn case3_thirty_two_bit() {
        // Tag 0b11, all 32-bit (selector 0b11 for each)
        // tags byte: 11_11_11_11 = 0xFF
        let b = [
            0xFF, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
        ];
        let mut b = Reader::new(&b);

        assert_eq!([1, 2, 3], tagged_32_variable(&mut b).unwrap());
        assert!(b.is_empty());
    }

    #[test]
    #[should_panic(expected = "Eof")]
    fn eof_case1() {
        let mut b = Reader::new(&[0x40]);
        tagged_32_variable(&mut b).unwrap();
    }

    #[test]
    #[should_panic(expected = "Eof")]
    fn eof_case2() {
        let mut b = Reader::new(&[0x80]);
        tagged_32_variable(&mut b).unwrap();
    }
}
