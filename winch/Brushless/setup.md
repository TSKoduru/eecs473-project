# STM32 ESC Control - Project Setup Guide

This guide explains how to configure an STM32 project in STM32CubeIDE to control a brushless DC motor via an ESC (Electronic Speed Controller).

## Wiring Connections

| ESC Wire | Connection | Notes |
|----------|------------|-------|
| Signal (white/yellow/orange) | PA8 (STM32) | PWM control signal |
| Ground (black/brown) | GND (STM32) | **MUST be connected** |
| +5V/BEC (red) | Not connected | ESC powered by LiPo, STM32 by USB |

**Important:** Always connect grounds between ESC and STM32, even if they have separate power sources. You can do this by 
connecting the GND on the BEC (The little thing hanging off the ESC) to a GND pin on the STM32.

Link to website that has datasheet with pinout:
- [NUCLEO-U545RE-Q Datasheet](https://www.st.com/en/evaluation-tools/nucleo-u545re-q.html#documentation)

Download the file called "UM3062 STM32U3/U5 Nucleo-64 boards (MB1841)"
Pinout is on page 36.

---

## STM32CubeIDE Configuration (.ioc file)

### 1. Create New Project

1. File → New → STM32 Project
2. Select your board (e.g., NUCLEO-U545RE-Q) in Board Selector
3. Name your project
4. Click "No" when asked to initialize all peripherals with default mode

### 2. Pin Configuration

#### PA8 - PWM Output (ESC Control Signal)
1. Locate pin **PA8** on the chip diagram
2. Click on PA8 and select **TIM1_CH1** from dropdown
3. Pin should turn yellow 

#### PA5 - LED Output (Status Indicator)
1. Locate pin **PA5** (onboard LED on NUCLEO boards)
2. Click on PA5 and select **GPIO_Output**
3. Optional: Right-click PA5 → "Enter User Label" → name it "LED"

### 3. Timer Configuration (TIM1)

Navigate to **Timers → TIM1** in the left panel.

#### Mode Settings:
- **Clock Source**: Internal Clock
- **Channel 1**: PWM Generation CH1

#### Parameter Settings Tab:

**Counter Settings:**
- **Prescaler (PSC)**: Calculate based on your clock
  - Formula: `(Timer_Clock / 1,000,000) - 1`
  - Example: 4MHz clock → Prescaler = 3
  - Example: 160MHz clock → Prescaler = 159
  - Goal: 1MHz timer frequency (1μs resolution)
- **Counter Mode**: Up
- **Counter Period (ARR)**: **19999**
  - Creates 20ms period (20,000 ticks at 1MHz = 20ms)
  - Results in 50Hz PWM frequency (required for ESCs)
- **Internal Clock Division**: No Division
- **Repetition Counter**: 0
- **Auto-reload preload**: Enable

**PWM Generation Channel 1:**
- **Mode**: PWM mode 1
- **Pulse (CCR1)**: **1000**
  - Initial pulse width of 1ms (0% throttle for ESC arming)
- **Output compare preload**: Enable
- **Fast Mode**: Disable
- **CH Polarity**: High

### 4. Clock Configuration

1. Navigate to **Clock Configuration** tab
2. Check the **APB2 Timer Clocks** frequency (this feeds TIM1)
3. Verify your prescaler calculation:
   - If APB2 Timer Clock = 4MHz → Prescaler = 3
   - If APB2 Timer Clock = 160MHz → Prescaler = 159
4. Optional: Increase SYSCLK to maximum (e.g., 160MHz) by typing in HCLK field

**Important:** The prescaler value depends on your actual timer clock frequency, not SYSCLK!

### 5. Generate Code

1. Save the .ioc file (Ctrl+S)
2. Click **Project → Generate Code** (or auto-generates on save)
3. Code will be generated in `main.c`
4. Overwrite with the provided example code to control the ESC!

---

## Understanding the Configuration

### Why 50Hz PWM?

ESCs use RC (Radio Control) standard PWM:
- **Frequency**: 50Hz (20ms period)
- **Pulse Width Range**: 1ms to 2ms
  - 1ms = 0% throttle (minimum/armed)
  - 1.5ms = 50% throttle
  - 2ms = 100% throttle (maximum)

The ESC reads the pulse width (time signal is HIGH) to determine motor speed.

### Timer Math Breakdown

With our configuration:
- Timer clock after prescaler: 1MHz (1μs per tick)
- ARR = 19999 → counts 0 to 19999 = 20,000 ticks
- Period = 20,000μs = 20ms = 50Hz ✓
- CCR range 1000-2000 → 1ms to 2ms pulse widths ✓

### Duty Cycle Clarification

ESC PWM uses very low duty cycles (5-10%):
- 1ms pulse / 20ms period = 5% duty cycle (min throttle)
- 2ms pulse / 20ms period = 10% duty cycle (max throttle)

This is normal! ESCs read absolute pulse width, not duty cycle percentage.

---

## Verification Checklist

Before running your code, verify:

- ✓ PA8 configured as TIM1_CH1 (alternate function)
- ✓ PA5 configured as GPIO_Output
- ✓ Prescaler calculated correctly for your timer clock
- ✓ ARR = 19999 (for 20ms period)
- ✓ CCR1 = 1000 (for 1ms initial pulse)
- ✓ PWM Mode 1 selected
- ✓ Channel 1 polarity set to High
- ✓ ESC ground connected to STM32 ground
- ✓ Motor propeller removed or secured for testing

---

## Testing the Signal

Use an oscilloscope or logic analyzer to verify:
- **Frequency**: ~50Hz
- **Pulse width**: Should vary from 1ms to 2ms during operation
- **Duty cycle**: ~5% to ~10%

If values are off, recheck your prescaler and ARR settings!

---

## Common Issues

### "PWM frequency is 4× too high"
- Check your ARR value - should be 19999, not 4999

### "Pulse widths are wrong"
- Verify prescaler calculation matches your actual timer clock
- Check Clock Configuration tab for APB2 Timer Clocks frequency

### "ESC not arming"
- Ensure ground is connected between ESC and STM32
- Check that initial CCR1 = 1000 (minimum throttle)
- Listen for ESC beeps during 5-second arming sequence
- Some ESCs require throttle calibration - check ESC manual

### "Motor spins immediately"
- ESC might not be properly armed
- Ensure 5-second delay at minimum throttle before ramping

---

## Next Steps

After completing this setup:
1. Build and flash your code
2. Power ESC with LiPo battery
3. Listen for ESC arming beeps (5-second blink sequence)
4. Motor should ramp up smoothly after arming
5. Adjust ramp speed or throttle range as needed in code

For reversing motor direction, simply swap any two of the three motor wires connected to the ESC.