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
        // DnB Snare
        let mut tone_freq = 600.0;
        let mut phase = 0.0;
        let mut noise_seed = 42u32;

        for i in 0..SNARE_SAMPLE_LEN {
            // 1. 音调部分
            let tone = phase.sin();
            tone_freq *= 0.996; // 下降速度适中
            let phase_inc = (2.0 * PI * tone_freq) / 48000.0;
            phase += phase_inc;
            if phase > 2.0 * PI { phase -= 2.0 * PI; }

            // 2. 噪音部分 (沙沙声)
            noise_seed = noise_seed.wrapping_mul(1664525).wrapping_add(1013904223);
            let noise = ((noise_seed as f32 / u32::MAX as f32) * 2.0) - 1.0;

            // 3. 混合：DnB Snare 需要更多的噪音 (Snap)
            // 前半段全是噪音，后半段露出一眯眯音调
            let noise_ratio = 0.8 + (i as f32 / SNARE_SAMPLE_LEN as f32) * 0.2; 
            let mixed = (tone * (1.0 - noise_ratio)) + (noise * noise_ratio);

            // 4. 激进的失真
            let drive = 4.0; // 增加失真
            let distorted = cheap_saturator(mixed * drive);

            // 5. 极短的包络 (Tight Envelope)
            // 只要那一瞬间的 "Pia!"
            let env_decay = (1.0 - (i as f32 / SNARE_SAMPLE_LEN as f32)).powf(3.0);
            
            table[i] = distorted * env_decay * 0.8;
        }
        info!("DnB Snare generated.");
        table
    })
}
pub fn get_hat_sample_table() -> &'static [f32; HAT_SAMPLE_LEN] {
    HAT_SAMPLE_TABLE.get_or_init(|| {
        let mut table = [0.0f32; HAT_SAMPLE_LEN];
        let mut noise_seed = 999u32;
        let mut last_noise_sample = 0.0;

        for i in 0..HAT_SAMPLE_LEN {
            // 高频噪音
            noise_seed = noise_seed.wrapping_mul(1664525).wrapping_add(1013904223);
            let noise = ((noise_seed as f32 / u32::MAX as f32) * 2.0) - 1.0;

            // 高通滤波 (HPF) - 保持不变，这个很好用
            let filtered_noise = noise - last_noise_sample; 
            last_noise_sample = noise;

            // 关键修改：包络变得极短 (Closed Hat)
            // 就像用鼓槌尖轻轻点了一下
            let env = (1.0 - (i as f32 / HAT_SAMPLE_LEN as f32)).powf(10.0); // 10次方！极快衰减

            table[i] = filtered_noise * env * 0.6;
        }
        info!("DnB Hat generated.");
        table
    })
}