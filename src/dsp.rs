use micromath::F32Ext;

// --- 浮点辅助 (用于波表生成的初始化阶段) ---

pub fn cheap_saturator(x: f32) -> f32 {
    // 1. 限制输入范围 [防止 x^3 运算时浮点溢出]
    // 限制在 [-1.5, 1.5] 是一个安全的范围，保证曲线平滑
    let input_clamped = x.clamp(-1.5, 1.5); 

    // 2. 软削波运算 (无除法，只有乘法)
    let x2 = input_clamped * input_clamped;
    let x3 = x2 * input_clamped;
    
    // 3. 应用公式: x - x^3 * (1/3)
    let output = input_clamped - x3 * 0.33333334;
    
    // 4. 最终输出硬限制，确保在 [-1.0, 1.0] 内
    // (这防止了 Cubic Clipper 在极端输入时略微超出 1.0)
    output.clamp(-1.0, 1.0)
}

// --- 定点数/整数 DSP 工具 (用于实时 Audio Loop) ---

/// 将 f32 (-1.0 ~ 1.0) 转换为 Q15 i16 (-32767 ~ 32767)
/// 用于参数控制信号转换
#[inline(always)]
pub fn f32_to_q15(x: f32) -> i16 {
    let x_clamped = x.clamp(-1.0, 1.0 - 1.0 / 32768.0);
    (x_clamped * 32768.0) as i16
}

/// Q15 乘法: (a * b) >> 15
/// 这是定点音频处理中最核心的运算，模拟了 1.0 * 0.5 = 0.5 的行为
#[inline(always)]
pub fn q15_mul(a: i16, b: i16) -> i16 {
    ((a as i32 * b as i32) >> 15) as i16
}

/// 纯整数 Bitcrusher
/// 这里的实现移除了所有浮点转换，速度极快
pub fn bitcrusher_i16(x: i16, bit: u8) -> i16 {
    // 如果位数 >= 16，直接返回原值
    if bit >= 16 {
        return x;
    }
    // 降低位深度的标准做法：右移丢弃低位，再左移补零
    let shift = (16 - bit).max(0);
    // 注意：对于有符号数，右移是算术右移 (保留符号位)
    (x >> shift) << shift
}

/// 简单的硬限制 (Hard Clip)
/// 用于最后的输出级保护
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

// (旧的 float 版本保留用于兼容，如果不需要可以删除)
pub fn bitcrusher_float(x: f32, bits: u8) -> f32 {
    if bits >= 16 {
        return x;
    }
    let delta = 2.0 / (1 << bits) as f32;
    let x_clamped = x.clamp(-1.0, 1.0);
    (x_clamped / delta).round() * delta
}