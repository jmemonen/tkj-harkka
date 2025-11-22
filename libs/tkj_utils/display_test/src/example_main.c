#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>

#include <tkjhat/sdk.h>
#include <sensors/message_output.h>

int main(void) {
    init_hat_sdk();      // Initialize I2C (SDA=@ref DEFAULT_I2C_SDA_PIN, SCL=@ref DEFAULT_I2C_SCL_PIN)
    init_display();          // Initialize SSD1306 (@ref SSD1306_I2C_ADDRESS — 0x3C)
 
    clear_display();         // Start clean
    write_text_xy(10, 20, "Hello!"); // Write text
 
    draw_circle(64, 32, 10, false);  // Draw circle outline
    draw_square(0, 0, 20, 20, true); // Draw filled rectangle
 
    while (true) { tight_loop_contents(); }
}