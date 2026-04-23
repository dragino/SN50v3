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
#ifndef __LORAMACTEST_H__
#define __LORAMACTEST_H__

/*!
 * \brief   Enabled or disables the reception windows
 *
 * \details This is a test function. It shall be used for testing purposes only.
 *          Changing this attribute may lead to a non-conformance LoRaMac operation.
 *
 * \param   [IN] enable - Enabled or disables the reception windows
 */
void LoRaMacTestRxWindowsOn( bool enable );

/*!
 * \brief   Enables the MIC field test
 *
 * \details This is a test function. It shall be used for testing purposes only.
 *          Changing this attribute may lead to a non-conformance LoRaMac operation.
 *
 * \param   [IN] txPacketCounter - Fixed Tx packet counter value
 */
void LoRaMacTestSetMic( uint16_t txPacketCounter );

/*!
 * \brief   Enabled or disables the duty cycle
 *
 * \details This is a test function. It shall be used for testing purposes only.
 *          Changing this attribute may lead to a non-conformance LoRaMac operation.
 *
 * \param   [IN] enable - Enabled or disables the duty cycle
 */
void LoRaMacTestSetDutyCycleOn( bool enable );

/*!
 * \brief   Sets the channel index
 *
 * \details This is a test function. It shall be used for testing purposes only.
 *          Changing this attribute may lead to a non-conformance LoRaMac operation.
 *
 * \param   [IN] channel - Channel index
 */
void LoRaMacTestSetChannel( uint8_t channel );

/*! \} defgroup LORAMACTEST */

#endif // __LORAMACTEST_H__
