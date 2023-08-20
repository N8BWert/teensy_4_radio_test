use embedded_hal::digital::v2::OutputPin;

use core::fmt::Debug;

#[derive(Debug, Clone, Copy)]
enum LedState {
    Red,
    Blue,
    Green,
    Off,
}

pub struct RGBLed<E: Debug, R: OutputPin<Error = E>, G: OutputPin<Error = E>, B: OutputPin<Error=E>> {
    red: R,
    green: G,
    blue: B,
    current_state: LedState,
    last_state: LedState,
}

impl<E: Debug, R: OutputPin<Error = E>, G: OutputPin<Error = E>, B: OutputPin<Error = E>> RGBLed<E, R, G, B> {
    pub fn new(red: R, green: G, blue: B) -> RGBLed<E, R, G, B> {
        Self { red, green, blue, current_state: LedState::Off, last_state: LedState::Off }
    }

    pub fn red(&mut self) -> Result<(), E> {
        self.red.set_high()?;
        self.green.set_low()?;
        self.blue.set_low()?;

        self.last_state = self.current_state;
        self.current_state = LedState::Red;

        Ok(())
    }

    pub fn green(&mut self) -> Result<(), E> {
        self.red.set_low()?;
        self.green.set_high()?;
        self.blue.set_low()?;

        self.last_state = self.current_state;
        self.current_state = LedState::Green;

        Ok(())
    }

    pub fn blue(&mut self) -> Result<(), E> {
        self.red.set_low()?;
        self.green.set_low()?;
        self.blue.set_high()?;

        self.last_state = self.current_state;
        self.current_state = LedState::Blue;

        Ok(())
    }

    pub fn off(&mut self) -> Result<(), E> {
        self.red.set_low()?;
        self.green.set_low()?;
        self.blue.set_low()?;

        self.last_state = self.current_state;
        self.current_state = LedState::Off;
        
        Ok(())
    }

    pub fn toggle(&mut self) -> Result<(), E> {
        match self.last_state {
            LedState::Red => self.red(),
            LedState::Green => self.green(),
            LedState::Blue => self.blue(),
            LedState::Off => self.off(),
        }
    }
}