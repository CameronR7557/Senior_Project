//Code from: https://github.com/pms67

#include "FIRFilter.h"

/*static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {-0.0032906f,-0.0052635f,-0.0068811f,0.0000000f,
														0.0254209f,0.0724719f,0.1311260f,
														0.1805961f,0.2000000f,0.1805961f,0.1311260f,0.0724719f,
														0.0254209f,0.0000000f,-0.0068811f,-0.0052635f};*/

//static float FIR_IMPULSE_RESPONSE[15] = {-0.0020337f,-0.0008993f,0.0039018f,0.0181146f,0.0447434f,0.0808759f,0.1175328f,0.1430680f,0.1485036f,0.1319102f,0.0993684f,0.0619557f,0.0304551f,0.0107140f,0.0020726f};

static float FIR_IMPULSE_RESPONSE[16] = {-0.0008604f,-0.0046728f,-0.0114995f,-0.0137994f,0.0054205f,0.0587142f,0.1376019f,0.2103260f,0.2400000f,0.2103260f,0.1376019f,0.0587142f,0.0054205f,-0.0137994f,-0.0114995f,-0.0046728f};
//static float FIR_IMPULSE_RESPONSE[15] = {0.0004336f,0.0027696f,0.0097290f,0.0243641f,0.0469532f,0.0740909f,0.0994500f,0.1160087f,0.1188029f,0.1069612f,0.0840802f,0.0567580f,0.0319592f,0.0144103f,0.0051679f};//this one so far

void FIRFilter_Init(FIRFilter *fir) {

	/* Clear filter buffer */
	for (uint8_t n = 0; n < FIR_FILTER_LENGTH; n++) {

		fir->buf[n] = 0.0f;

	}

	/* Reset buffer index */
	fir->bufIndex = 0;

	/* Clear filter output */
	fir->out = 0.0f;

}

float FIRFilter_Update(FIRFilter *fir, float inp) {

	/* Store latest sample in buffer */
	fir->buf[fir->bufIndex] = inp;

	/* Increment buffer index and wrap around if necessary */
	fir->bufIndex++;

	if (fir->bufIndex == FIR_FILTER_LENGTH) {

		fir->bufIndex = 0;

	}

	/* Compute new output sample (via convolution) */
	fir->out = 0.0f;

	uint8_t sumIndex = fir->bufIndex;

	for (uint8_t n = 0; n < FIR_FILTER_LENGTH; n++) {

		/* Decrement index and wrap if necessary */
		if (sumIndex > 0) {

			sumIndex--;

		} else {

			sumIndex = FIR_FILTER_LENGTH - 1;

		}

		/* Multiply impulse response with shifted input sample and add to output */
		fir->out += FIR_IMPULSE_RESPONSE[n] * fir->buf[sumIndex];

	}

	/* Return filtered output */
	return fir->out;

}
