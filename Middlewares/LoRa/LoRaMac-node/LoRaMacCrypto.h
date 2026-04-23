/*
	MIT License

	Copyright (c) 2026 DRAGINO Team

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/
#ifndef __LORAMAC_CRYPTO_H__
#define __LORAMAC_CRYPTO_H__

/*!
 * Computes the LoRaMAC frame MIC field
 *
 * \param [IN]  buffer          - Data buffer
 * \param [IN]  size            - Data buffer size
 * \param [IN]  key             - AES key to be used
 * \param [IN]  address         - Frame address
 * \param [IN]  dir             - Frame direction [0: uplink, 1: downlink]
 * \param [IN]  sequenceCounter - Frame sequence counter
 * \param [OUT] mic             - Computed MIC field
 */
void LoRaMacComputeMic( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint32_t *mic );

/*!
 * Computes the LoRaMAC payload encryption
 *
 * \param [IN]  buffer          - Data buffer
 * \param [IN]  size            - Data buffer size
 * \param [IN]  key             - AES key to be used
 * \param [IN]  address         - Frame address
 * \param [IN]  dir             - Frame direction [0: uplink, 1: downlink]
 * \param [IN]  sequenceCounter - Frame sequence counter
 * \param [OUT] encBuffer       - Encrypted buffer
 */
void LoRaMacPayloadEncrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint8_t *encBuffer );

/*!
 * Computes the LoRaMAC payload decryption
 *
 * \param [IN]  buffer          - Data buffer
 * \param [IN]  size            - Data buffer size
 * \param [IN]  key             - AES key to be used
 * \param [IN]  address         - Frame address
 * \param [IN]  dir             - Frame direction [0: uplink, 1: downlink]
 * \param [IN]  sequenceCounter - Frame sequence counter
 * \param [OUT] decBuffer       - Decrypted buffer
 */
void LoRaMacPayloadDecrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint8_t *decBuffer );

/*!
 * Computes the LoRaMAC Join Request frame MIC field
 *
 * \param [IN]  buffer          - Data buffer
 * \param [IN]  size            - Data buffer size
 * \param [IN]  key             - AES key to be used
 * \param [OUT] mic             - Computed MIC field
 */
void LoRaMacJoinComputeMic( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t *mic );

/*!
 * Computes the LoRaMAC join frame decryption
 *
 * \param [IN]  buffer          - Data buffer
 * \param [IN]  size            - Data buffer size
 * \param [IN]  key             - AES key to be used
 * \param [OUT] decBuffer       - Decrypted buffer
 */
void LoRaMacJoinDecrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint8_t *decBuffer );

/*!
 * Computes the LoRaMAC join frame decryption
 *
 * \param [IN]  key             - AES key to be used
 * \param [IN]  appNonce        - Application nonce
 * \param [IN]  devNonce        - Device nonce
 * \param [OUT] nwkSKey         - Network session key
 * \param [OUT] appSKey         - Application session key
 */
void LoRaMacJoinComputeSKeys( const uint8_t *key, const uint8_t *appNonce, uint16_t devNonce, uint8_t *nwkSKey, uint8_t *appSKey );

/*! \} defgroup LORAMAC */

#endif // __LORAMAC_CRYPTO_H__
