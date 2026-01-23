use micromath::F32Ext;


pub fn cheap_saturator(x: f32) -> f32 {
    let input_clamped = x.clamp(-1.5, 1.5); 

    let x2 = input_clamped * input_clamped;
    let x3 = x2 * input_clamped;
    
    let output = input_clamped - x3 * 0.33333334;
    
    output.clamp(-1.0, 1.0)
}

#[inline(always)]
pub fn f32_to_q15(x: f32) -> i16 {
    let x_clamped = x.clamp(-1.0, 1.0 - 1.0 / 32768.0);
    (x_clamped * 32768.0) as i16
}


#[inline(always)]
pub fn q15_mul(a: i16, b: i16) -> i16 {
    ((a as i32 * b as i32) >> 15) as i16
}

pub fn bitcrusher_i16(x: i16, bit: u8) -> i16 {
    if bit >= 16 {
        return x;
    }
    let shift = (16 - bit).max(0);
    (x >> shift) << shift
}

#[inline(always)]
pub fn saturate_i16(x: i32) -> i16 {
    if x > 32767 {
        32767
    } else if x < -32768 {
        -32768
    } else {
        x as i16
    }
}

pub fn bitcrusher_float(x: f32, bits: u8) -> f32 {
    if bits >= 16 {
        return x;
    }
    let delta = 2.0 / (1 << bits) as f32;
    let x_clamped = x.clamp(-1.0, 1.0);
    (x_clamped / delta).round() * delta
}