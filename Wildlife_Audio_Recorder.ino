#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <driver/i2s.h>

// --- Configuration ---
#define I2S_WS 15
#define I2S_SD 32
#define I2S_SCK 14
#define I2S_PORT I2S_NUM_0

// SD Card CS Pin
#define SD_CS 5

// Recording Settings
#define SAMPLE_RATE 26000 // 16kHz (Good for voice/animals)
#define RECORD_TIME 30    // Seconds to record per file
#define I2S_BUFFER_SIZE 1024

File file;
const int headerSize = 44;

void i2s_install() {
  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // INMP441 L/R grounded
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = I2S_BUFFER_SIZE,
    .use_apll = false
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin() {
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };
  i2s_set_pin(I2S_PORT, &pin_config);
}

// Function to write WAV header
void wavHeader(File file, int sampleRate, int dataSize) {
  int byteRate = sampleRate * 2; // 16 bit = 2 bytes
  byte header[headerSize];

  header[0] = 'R'; header[1] = 'I'; header[2] = 'F'; header[3] = 'F';
  unsigned int fileSize = dataSize + headerSize - 8;
  header[4] = (byte)(fileSize & 0xFF);
  header[5] = (byte)((fileSize >> 8) & 0xFF);
  header[6] = (byte)((fileSize >> 16) & 0xFF);
  header[7] = (byte)((fileSize >> 24) & 0xFF);
  header[8] = 'W'; header[9] = 'A'; header[10] = 'V'; header[11] = 'E';
  header[12] = 'f'; header[13] = 'm'; header[14] = 't'; header[15] = ' ';
  header[16] = 16; header[17] = 0; header[18] = 0; header[19] = 0;
  header[20] = 1; header[21] = 0;
  header[22] = 1; header[23] = 0; // 1 Channel (Mono)
  header[24] = (byte)(sampleRate & 0xFF);
  header[25] = (byte)((sampleRate >> 8) & 0xFF);
  header[26] = (byte)((sampleRate >> 16) & 0xFF);
  header[27] = (byte)((sampleRate >> 24) & 0xFF);
  header[28] = (byte)(byteRate & 0xFF);
  header[29] = (byte)((byteRate >> 8) & 0xFF);
  header[30] = (byte)((byteRate >> 16) & 0xFF);
  header[31] = (byte)((byteRate >> 24) & 0xFF);
  header[32] = 2; header[33] = 0; // Block align
  header[34] = 16; header[35] = 0; // Bits per sample
  header[36] = 'd'; header[37] = 'a'; header[38] = 't'; header[39] = 'a';
  header[40] = (byte)(dataSize & 0xFF);
  header[41] = (byte)((dataSize >> 8) & 0xFF);
  header[42] = (byte)((dataSize >> 16) & 0xFF);
  header[43] = (byte)((dataSize >> 24) & 0xFF);

  file.write(header, headerSize);
}

void setup() {
  Serial.begin(115200);
  
  // Initialize SD Card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card Mount Failed!");
    return;
  }
  Serial.println("SD Card Initialized.");

  // Initialize I2S
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
  
  delay(1000); // Wait a bit
}

void loop() {
  // Generate a new filename based on millis to avoid overwriting
  String filename = "/rec_" + String(millis()) + ".wav";
  Serial.print("Recording to: ");
  Serial.println(filename);

  file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  // Reserve space for header
  byte header[headerSize]; 
  file.write(header, headerSize);

  int flashWrSize = 0;
  size_t bytesRead;
  // Buffer for reading I2S data
  char* i2s_read_buff = (char*) calloc(I2S_BUFFER_SIZE, sizeof(char));

  unsigned long startTime = millis();
  
  // Recording Loop
  while (millis() - startTime < (RECORD_TIME * 1000)) {
    // Read from Microphone
    i2s_read(I2S_PORT, (void*)i2s_read_buff, I2S_BUFFER_SIZE, &bytesRead, portMAX_DELAY);
    
    // Write to SD Card
    file.write((const uint8_t*)i2s_read_buff, bytesRead);
    flashWrSize += bytesRead;
  }

  // Update WAV Header with final file size
  file.seek(0);
  wavHeader(file, SAMPLE_RATE, flashWrSize);
  
  file.close();
  free(i2s_read_buff);
  
  Serial.println("Recording Finished.");
  delay(1000); // Wait 1 second before next recording
}
