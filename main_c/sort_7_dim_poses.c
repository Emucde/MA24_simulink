#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* THIS FILE IS 100% AI GENERATED FROM https://www.perplexity.ai/*/

// Function to calculate the Euclidean norm
double calculate_norm(double *pose1, double *pose2, int length) {
    double sum = 0.0;
    for (int i = 0; i < length; i++) {
        sum += pow(pose1[i] - pose2[i], 2);
    }
    return sqrt(sum);
}

// Function to count rows and columns in a CSV file
void count_csv_dimensions(const char *filename, int *rows, int *cols) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        perror("Error opening file");
        exit(EXIT_FAILURE);
    }

    char *line = NULL;
    size_t len = 0;
    ssize_t read;

    *rows = 0;
    *cols = 0;

    while ((read = getline(&line, &len, file)) != -1) {
        (*rows)++;
        if (*cols == 0) {
            // Count columns only for the first row
            char *temp_line = strdup(line); // Duplicate line for tokenization
            char *token = strtok(temp_line, ",");
            while (token) {
                (*cols)++;
                token = strtok(NULL, ",");
            }
            free(temp_line);
        }
    }

    free(line);
    fclose(file);
}

// Function to read CSV data into a matrix
void read_csv(const char *filename, double **data, int rows, int cols) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        perror("Error opening file");
        exit(EXIT_FAILURE);
    }

    char *line = NULL;
    size_t len = 0;
    ssize_t read;

    int row = 0;

    while ((read = getline(&line, &len, file)) != -1) {
        int col = 0;
        char *token = strtok(line, ",");
        while (token) {
            data[row][col++] = atof(token);
            token = strtok(NULL, ",");
        }
        row++;
    }

    free(line);
    fclose(file);
}

// Function to write data to a CSV file
void write_csv(const char *filename, double **data, int rows, int cols) {
    FILE *file = fopen(filename, "w");
    if (!file) {
        perror("Error opening file");
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            fprintf(file, "%f", data[i][j]);
            if (j < cols - 1) fprintf(file, ",");
        }
        fprintf(file, "\n");
    }
    fclose(file);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <input_csv_file>\n", argv[0]);
        return EXIT_FAILURE;
    }

    const char *input_file = argv[1];
    const char *output_file = "sorted_output.csv";

    // Detect number of rows and columns in the input CSV
    int rows, cols;
    count_csv_dimensions(input_file, &rows, &cols);

    // Ensure that the input file has exactly 7 columns
    if (cols != 7) {
        fprintf(stderr, "Error: The input file must contain exactly 7 columns for 7-dimensional poses.\n");
        return EXIT_FAILURE;
    }

    printf("Detected %d rows and %d columns in the input file.\n", rows, cols);

    // Allocate memory for the data matrix
    double **data = (double **)malloc(rows * sizeof(double *));
    for (int i = 0; i < rows; i++) {
        data[i] = (double *)malloc(cols * sizeof(double));
    }

    // Read the input CSV into the data matrix
    read_csv(input_file, data, rows, cols);

    // Sorting poses based on distances
    double **sorted_pose = (double **)malloc(rows * sizeof(double *));
    for (int i = 0; i < rows; i++) {
        sorted_pose[i] = (double *)malloc(cols * sizeof(double));
        memset(sorted_pose[i], 0, cols * sizeof(double));
    }

    double pose_distances[rows];
    
    memcpy(sorted_pose[0], data[0], cols * sizeof(double)); // Initialize sorted_pose with the first pose

    int remaining_rows = rows; // Track remaining rows separately

    for (int i = 0; i < rows - 1; i++) { // Iterate over all original rows
        double min_distance = INFINITY;
        int min_idx = -1;

        for (int j = 1; j < remaining_rows; j++) { // Find closest pose in remaining poses
            double distance = calculate_norm(sorted_pose[i], data[j], cols);
            if (distance < min_distance) {
                min_distance = distance;
                min_idx = j;
            }
        }

        pose_distances[i + 1] = min_distance;

        // Add closest pose to sorted_pose
        memcpy(sorted_pose[i + 1], data[min_idx], cols * sizeof(double));

        // Remove min_idx from data by overwriting it with the last element
        memcpy(data[min_idx], data[remaining_rows - 1], cols * sizeof(double));

        remaining_rows--; // Reduce size of remaining poses

        if (i % 100 == 0) { // Debugging output every 100 iterations
            printf("Iteration: %d\n", i);
            printf("Pose %d, pose_distances: %g\n", i + 1, pose_distances[i + 1]);
        }
    }

    // Write sorted poses to output CSV
    write_csv(output_file, sorted_pose, rows, cols);

    printf("Sorted output written to %s\n", output_file);

    // Free allocated memory
    for (int i = 0; i < rows; i++) free(data[i]);
    free(data);

    for (int i = 0; i < rows; i++) free(sorted_pose[i]);
    free(sorted_pose);

    return EXIT_SUCCESS;
}

