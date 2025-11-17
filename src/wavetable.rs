use core::f32::consts::PI;
use defmt::info;
use micromath::F32Ext;
use once_cell::sync::OnceCell;

// --- 常量和静态存储 ---
pub const WAVE_TABLE_SIZE: usize = 1024;

pub const KICK_SAMPLE_LEN: usize = 2048;
pub const SNARE_SAMPLE_LEN: usize = 1500;
pub const HAT_SAMPLE_LEN: usize = 1000;

static SINE_TABLE: OnceCell<[f32; WAVE_TABLE_SIZE]> = OnceCell::new();
static SQUARE_TABLE: OnceCell<[f32; WAVE_TABLE_SIZE]> = OnceCell::new();
static SAWTOOTH_TABLE: OnceCell<[f32; WAVE_TABLE_SIZE]> = OnceCell::new();
static TRIANGLE_TABLE: OnceCell<[f32; WAVE_TABLE_SIZE]> = OnceCell::new();

static KICK_SAMPLE_TABLE: OnceCell<[f32; KICK_SAMPLE_LEN]> = OnceCell::new();
static SNARE_SAMPLE_TABLE: OnceCell<[f32; SNARE_SAMPLE_LEN]> = OnceCell::new();
static HAT_SAMPLE_TABLE: OnceCell<[f32; HAT_SAMPLE_LEN]> = OnceCell::new();

// --- 结构体和枚举 ---
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Waveform {
    Sine,
    Triangle,
    Sawtooth,
    Square,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WaveParams {
    pub carrier_wave: Waveform,
    pub mod_wave: Waveform,
}

// --- 辅助函数 ---
fn cheap_saturator(x: f32) -> f32 {
    x / (1.0 + x.abs())
}

// --- 波表生成函数 ---
pub fn get_sine_table() -> &'static [f32; WAVE_TABLE_SIZE] {
    SINE_TABLE.get_or_init(|| {
        let mut table = [0.0f32; WAVE_TABLE_SIZE];
        for i in 0..WAVE_TABLE_SIZE {
            let phase = (i as f32 / WAVE_TABLE_SIZE as f32) * (2.0 * PI);
            table[i] = phase.sin();
        }
        info!("Sine table (1024 samples) generated.");
        table
    })
}

pub fn get_square_table() -> &'static [f32; WAVE_TABLE_SIZE] {
    SQUARE_TABLE.get_or_init(|| {
        let mut table = [0.0f32; WAVE_TABLE_SIZE];
        for i in 0..WAVE_TABLE_SIZE {
            let phase = (i as f32 / WAVE_TABLE_SIZE as f32) * (2.0 * PI);
            table[i] = if phase < PI { 1.0 } else { -1.0 };
        }
        info!("Square table (1024 samples) generated.");
        table
    })
}

pub fn get_sawtooth_table() -> &'static [f32; WAVE_TABLE_SIZE] {
    SAWTOOTH_TABLE.get_or_init(|| {
        let mut table = [0.0f32; WAVE_TABLE_SIZE];
        for i in 0..WAVE_TABLE_SIZE {
            let normalized = i as f32 / WAVE_TABLE_SIZE as f32;
            table[i] = 2.0 * normalized - 1.0;
        }
        info!("Sawtooth table (1024 samples) generated.");
        table
    })
}

pub fn get_triangle_table() -> &'static [f32; WAVE_TABLE_SIZE] {
    TRIANGLE_TABLE.get_or_init(|| {
        let mut table = [0.0f32; WAVE_TABLE_SIZE];
        let half = WAVE_TABLE_SIZE / 2;
        for i in 0..WAVE_TABLE_SIZE {
            if i < half {
                let normalized = i as f32 / half as f32;
                table[i] = 2.0 * normalized - 1.0;
            } else {
                let normalized = (i - half) as f32 / half as f32;
                table[i] = 1.0 - 2.0 * normalized;
            }
        }
        info!("Triangle table (1024 samples) generated.");
        table
    })
}

// --- 鼓采样生成函数 ---
pub fn get_kick_sample_table() -> &'static [f32; KICK_SAMPLE_LEN] {
    KICK_SAMPLE_TABLE.get_or_init(|| {
        let mut table = [0.0f32; KICK_SAMPLE_LEN];
        let mut freq = 200.0; // 起始频率
        let mut phase = 0.0;

        for i in 0..KICK_SAMPLE_LEN {
            // 1. 振荡器 (Sine)
            let sample = phase.sin();

            let drive = 3.0; // 提升 3 倍增益
            let distorted_sample = cheap_saturator(sample * drive);

            // 2. 音高包络 (非常快地下降)
            freq *= 0.999; // 频率指数衰减
            let phase_inc = (2.0 * PI * freq) / 48000.0;
            phase += phase_inc;
            if phase > (2.0 * PI) {
                phase -= 2.0 * PI;
            }

            // 3. 音量包络 (线性衰减)
            let amp_env = 1.0 - (i as f32 / KICK_SAMPLE_LEN as f32);

            table[i] = distorted_sample * amp_env;
        }
        info!("Kick sample table (2048 samples) generated.");
        table
    })
}
pub fn get_snare_sample_table() -> &'static [f32; SNARE_SAMPLE_LEN] {
    SNARE_SAMPLE_TABLE.get_or_init(|| {
        let mut table = [0.0f32; SNARE_SAMPLE_LEN];
        
        // (修正 1: 提高初始频率，增强“裂开”感)
        // 原值: 200.0 -> 新值: 300.0
        let mut tone_freq = 300.0; // 军鼓基础音高（高于底鼓） 
        
        let mut phase = 0.0;
        let mut noise_seed = 42u32; // 用于生成伪随机噪音的种子

        for i in 0..SNARE_SAMPLE_LEN {
            // 1. 低频音调成分（正弦波，快速降调）
            let tone = phase.sin();
            
            // (修正 2: 减慢音高衰减速度，延长 Snare 的身躯)
            // 原值: 0.997 -> 新值: 0.998
            tone_freq *= 0.998; // 音高衰减速度
            
            let phase_inc = (2.0 * PI * tone_freq) / 48000.0;
            phase += phase_inc;
            if phase > 2.0 * PI {
                phase -= 2.0 * PI;
            }

            // 2. 噪音成分（白噪音，提供"沙沙声"）
            noise_seed = noise_seed.wrapping_mul(1664525).wrapping_add(1013904223);
            let noise = ((noise_seed as f32 / u32::MAX as f32) * 2.0) - 1.0; 

            // 3. 噪音与音调混合（随时间变化比例）
            let noise_ratio = 0.7 + (i as f32 / SNARE_SAMPLE_LEN as f32) * 0.3; 
            let mixed = (tone * (1.0 - noise_ratio)) + (noise * noise_ratio);

            // 4. 失真处理（增强金属感）
            let drive = 2.5;
            let distorted = cheap_saturator(mixed * drive);

            // 5. 音量包络（前10%快速上升，后90%快速衰减）
            let env_attack = (i as f32 / (SNARE_SAMPLE_LEN as f32 * 0.1)).min(1.0);
            let env_decay = 1.0 - (i as f32 / SNARE_SAMPLE_LEN as f32);
            let amp_env = env_attack * env_decay;

            table[i] = distorted * amp_env * 0.8; // 降低整体增益避免削波
        }
        info!("Snare sample table (1500 samples) generated.");
        table
    })
}
pub fn get_hat_sample_table() -> &'static [f32; HAT_SAMPLE_LEN] {
    HAT_SAMPLE_TABLE.get_or_init(|| {
        let mut table = [0.0f32; HAT_SAMPLE_LEN];
        let mut noise_seed = 123u32;
        let mut last_noise_sample = 0.0; // <-- (新!) 用于 HPF 的状态变量

        for i in 0..HAT_SAMPLE_LEN {
            // 1. 高频噪音生成 (LCG)
            noise_seed = noise_seed.wrapping_mul(1664525).wrapping_add(1013904223);
            let noise = ((noise_seed as f32 / u32::MAX as f32) * 2.0) - 1.0; // 映射到[-1,1]

            // 2. (新!) 高通滤波模拟 (HPF)
            let filtered_noise = noise - last_noise_sample; // 核心：当前减去前一个样本
            last_noise_sample = noise;                      // 更新状态

            // 3. 快速衰减包络 (保持不变)
            let env = if i < HAT_SAMPLE_LEN / 20 {
                (i as f32 * 20.0) / HAT_SAMPLE_LEN as f32
            } else {
                (1.0 - (i as f32 / HAT_SAMPLE_LEN as f32)).powf(2.0)
            };

            // 应用包络到滤波后的噪音
            table[i] = filtered_noise * env * 0.5;
        }
        info!("Hat sample table generated.");
        table
    })
}