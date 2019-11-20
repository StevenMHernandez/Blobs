#ifndef BLOBS_RFID_H
#define BLOBS_RFID_H

#include <Adafruit_PN532.h>

#define PN532_IRQ   (4)
#define PN532_SCK  (13)
#define PN532_MOSI (11)
#define PN532_SS   (9)
#define PN532_MISO (12)

Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

bool rfid_tag_present = false;
void handleInterruptFalling() {
    rfid_tag_present = true;
}

void setup_rfid() {
    pinMode(PN532_IRQ, INPUT_PULLUP);
    nfc.startPassiveTargetIDDetection(PN532_MIFARE_ISO14443A);
    attachInterrupt(digitalPinToInterrupt(PN532_IRQ), handleInterruptFalling, FALLING);
    nfc.begin();
    nfc.SAMConfig();
}

char filename[50];
void rfid_update(int t, char msg[], boolean *message_exists) {
    uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0};
    uint8_t uidLength;

    if (rfid_tag_present) {
        nfc.readDetectedPassiveTargetID(uid, &uidLength);

        char filename[50];
        sprintf(filename, "0x%02X/0x%02X/0x%02X/0x%02X/0x%02X/0x%02X/0x%02X/msg.txt", uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6]);

        Serial.print("opening: ");
        Serial.println(filename);

        SdFile rdfile(filename, O_RDONLY);
        if (!rdfile.isOpen()) {
            Serial.println("can't open file. Does it exist?");
            sprintf(msg, "Unknown:%s", filename);
            *message_exists = true;
        } else {
            rdfile.fgets(msg, 100 * sizeof(char));
            *message_exists = true;
        }
        Serial.println(msg);

        nfc.startPassiveTargetIDDetection(PN532_MIFARE_ISO14443A);
        rfid_tag_present = false;
    } else {
        *message_exists = false;
    }

    if (t == 0) {
        *message_exists = false;
    }
}

#endif //BLOBS_RFID_H
