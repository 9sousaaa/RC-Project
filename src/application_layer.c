// RCOM 2026/2027
// Application layer protocol implementation
#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define CTRL_DATA 1
#define CTRL_START 2
#define CTRL_END 3

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer linkLayer;
    strncpy(linkLayer.serialPort, serialPort, sizeof(linkLayer.serialPort) - 1);
    linkLayer.serialPort[sizeof(linkLayer.serialPort) - 1] = '\0';
    linkLayer.baudRate = baudRate;
    linkLayer.nRetransmissions = nTries;
    linkLayer.timeout = timeout;

    if (strcmp(role, "tx") == 0) {
        if (llOpenTx(linkLayer) < 0) {
            printf("Erro a abrir conexao (Tx).\n");
            return;
        }
        FILE *f = fopen(filename, "rb");
        if (!f) { perror("fopen"); llCloseTx(); return; }
        
        fseek(f, 0, SEEK_END);
        long file_size = ftell(f);
        rewind(f);

        unsigned char cp[256];
        cp[0] = CTRL_START;
        cp[1] = 0; // Type Size
        cp[2] = 4; // Length
        cp[3] = (file_size >> 24) & 0xFF;
        cp[4] = (file_size >> 16) & 0xFF;
        cp[5] = (file_size >> 8) & 0xFF;
        cp[6] = file_size & 0xFF;
        cp[7] = 1; // Type Name
        int len_name = strlen(filename);
        if (len_name > 200) len_name = 200;   // cabe em cp[256] e num byte (L)
        cp[8] = len_name;
        memcpy(&cp[9], filename, len_name);
        
        int cp_size = 9 + len_name;
        if (llSend(cp, cp_size) < 0) {
            printf("Erro ao enviar pacote de controlo START.\n");
            fclose(f);
            llCloseTx();
            return;
        }

        unsigned char buf[800];
        int bytes_read;
        int seq = 0;
        
        while ((bytes_read = fread(buf, 1, 800, f)) > 0) {
            unsigned char* dp = malloc(bytes_read + 4);
            dp[0] = CTRL_DATA;
            dp[1] = seq % 256;
            dp[2] = (bytes_read >> 8) & 0xFF;
            dp[3] = bytes_read & 0xFF;
            memcpy(&dp[4], buf, bytes_read);
            
            if (llSend(dp, bytes_read + 4) < 0) {
                printf("Erro ao enviar dados.\n");
                free(dp);
                break;
            }
            free(dp);
            seq++;
        }

        int ok = (bytes_read == 0 && !ferror(f));
        if (ok) {
            cp[0] = CTRL_END;
            if (llSend(cp, cp_size) < 0) ok = 0;
        }
        fclose(f);
        llCloseTx();
        if (ok) printf("Ficheiro enviado com sucesso!\n");
        else printf("Falha no envio do ficheiro.\n");
        
    } else if (strcmp(role, "rx") == 0) {
        if (llOpenRx(linkLayer) < 0) {
            printf("Erro a abrir conexao (Rx).\n");
            return;
        }
        
        FILE *f = fopen(filename, "wb");
        if (!f) { perror("fopen"); llCloseRx(); return; }
        
        unsigned char packet[MAX_PAYLOAD_SIZE];
        while (1) {
            int n = llReceive(packet);
            if (n > 0) {
                if (packet[0] == CTRL_START) continue;
                else if (packet[0] == CTRL_DATA) {
                    int p_size = (packet[2] << 8) | packet[3];
                    if (n < 4 || p_size > n - 4) continue;   // pacote inconsistente
                    fwrite(&packet[4], 1, p_size, f);
                }
                else if (packet[0] == CTRL_END) break;
            }
        }
        fclose(f);
        llCloseRx();
        printf("Ficheiro recebido com sucesso!\n");
    }
}