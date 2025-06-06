#ifndef TOUCH_H
#define TOUCH_H

#include <stdint.h>

// Define dimensions for selecting touchscreen sampling
#define TOUCH_DIM_X 0
#define TOUCH_DIM_Y 1

/**
 * @brief Initialize the touchscreen interface.
 *
 * Configures the ADC for 10-bit integer conversion, sets the proper ADC input pins
 * (AN15 for X and AN9 for Y), and performs any necessary pin configuration.
 */
void touch_init(void);

/**
 * @brief Select the touchscreen dimension to sample.
 *
 * @param dimension Use TOUCH_DIM_X for X-dimension or TOUCH_DIM_Y for Y-dimension.
 */
void touch_select_dim(uint8_t dimension);

/**
 * @brief Acquire a touchscreen sample.
 *
 * @return uint16_t The ADC conversion result from the selected dimension.
 */
uint16_t touch_read(void);

#endif /* TOUCH_H */
