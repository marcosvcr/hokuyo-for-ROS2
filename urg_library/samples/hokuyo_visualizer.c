#include "urg_sensor.h"
#include "urg_utils.h"
#include <stdio.h>
#include <stdlib.h>

int main() {
    urg_t urg;
    int ret;
    long *data;
    int data_size;

    // Connect to the Hokuyo LiDAR
    const char *device = "/dev/ttyACM0";
    int baudrate = 115200;

    ret = urg_open(&urg, URG_SERIAL, device, baudrate);
    if (ret < 0) {
        printf("Error: %s\n", urg_error(&urg));
        return EXIT_FAILURE;
    }

    // Allocate memory for scan data
    data_size = urg_max_data_size(&urg);
    data = (long *)malloc(data_size * sizeof(long));

    // Start measurement
    urg_start_measurement(&urg, URG_DISTANCE, 1, 0);

    // Get scan data
    int n = urg_get_distance(&urg, data, NULL);
    if (n < 0) {
        printf("Error: %s\n", urg_error(&urg));
        free(data);
        urg_close(&urg);
        return EXIT_FAILURE;
    }

    // Print scan data
    for (int i = 0; i < n; ++i) {
        printf("%ld\n", data[i]);
    }

    // Clean up
    free(data);
    urg_close(&urg);
    return EXIT_SUCCESS;
}

