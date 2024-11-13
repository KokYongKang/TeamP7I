#ifndef BARCODE_H
#define BARCODE_H

#define MAX_BUFFER_SIZE 1024

void initialize_motors();
int decode_binary_pattern(const char *pattern);
void analyze_and_compile_pattern();
void gpio_callback(uint gpio, uint32_t events);

#endif // BARCODE_H