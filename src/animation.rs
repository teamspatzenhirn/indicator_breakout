use core::cmp::min;
use smart_leds::{colors, RGB8};

use indicator_interface::IndicatorState;

pub const FRAME_DURATION: u32 = 500;
pub const STRIP_LEN: usize = 16 * 3;

const HOLD_DURATION: u32 = 250;
const FADE_DURATION: u32 = FRAME_DURATION - HOLD_DURATION;

pub struct AnimationState {
    pub mode: IndicatorState,
    pub frames_drawn: i16,
}

impl AnimationState {
    pub fn off() -> AnimationState {
        AnimationState {
            mode: IndicatorState::OFF,
            frames_drawn: 0,
        }
    }
}

pub fn calculate_frame(buf: &mut [RGB8; STRIP_LEN], t: u32, mode: &IndicatorState, brake: bool) {
    buf.fill(colors::BLACK);

    match mode {
        IndicatorState::LEFT => {
            let progress = min(16, 16 * t / FADE_DURATION) as usize;
            buf[32..32 + progress].fill(colors::YELLOW);
        }
        IndicatorState::RIGHT => {
            let progress = min(16, 16 * t / FADE_DURATION) as usize;
            buf[16 - progress..16].fill(colors::YELLOW);
        }
        IndicatorState::BOTH => {
            buf[0..16].fill(if t < FRAME_DURATION / 2 {
                colors::YELLOW
            } else {
                colors::BLACK
            });

            buf[STRIP_LEN - 16..STRIP_LEN].fill(if t < FRAME_DURATION / 2 {
                colors::YELLOW
            } else {
                colors::BLACK
            });
        }
        IndicatorState::OFF => {}
    }

    if brake {
        buf[0..3].fill(colors::RED);
        buf[STRIP_LEN - 3..STRIP_LEN].fill(colors::RED);
        buf[16..32].fill(colors::RED);
    }
}
