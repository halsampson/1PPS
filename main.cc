// F5328

// Check oscillators Hz using GPS 1 PPS

// 1PPS has extra pulses and missing pulses

// TODO: 55 MHz ovfls? ~ 1.8 change;  chk with 5340
// happened after 2:13 ~8000s

// TTL overshoot latch-up?

#include <msp430.h> 
#include <stdlib.h>
#include <string.h>

typedef unsigned char uchar;
typedef unsigned int  uint;
typedef unsigned long ulong;
typedef unsigned long long ullong;
typedef long long llong;

#define min(a,b) ((a) < (b) ? (a) : (b))

const ulong BaudRate  = 921600;
const ulong CPUHz = 4 * (8 * BaudRate);  // 8 * 921600 = 225 * 32768
const ulong XtalHz = 32768;

// P1 pins
#define XtalA0   BIT0  // TA1CLK  // pin 18
#define OnePPSA0 BIT2  // CCI0A   // pin 20
#define XtalA1   BIT6  // TA1CLK  // pin 24

// P2 pins
#define OnePPSA1 BIT0  // CCI1A   // pin 26
#define XtalA2   BIT2  // TA2CLK  // pin 28
#define OnePPSA2 BIT4  // CCI2A   // pin 30

// P3 pins:
#define TxD			  BIT3  // pin 37
#define RxD			  BIT4  // pin 38

// P4 pins
#define OnePPSB  BIT7  // pin 48 - corner

// P5 pins:
#define XIN 		  BIT4
#define XOUT	    BIT5



void pause() {  // run to here
	__delay_cycles(1);
}

const int SerBufSize = 8; // TODO: larger if long params from PC
char serBuf[SerBufSize];
const char* SerBufEnd = serBuf + SerBufSize;
char* serInPtr = serBuf;
char* serOutPtr = serBuf;

#pragma vector=USCI_A0_VECTOR
__interrupt void UART_RX_HOOK() {
	*serInPtr++ = UCA0RXBUF;
	if (serInPtr >= SerBufEnd)
		serInPtr = serBuf;

	switch (UCA0RXBUF) {
		case 'b' : 	WDTCTL = WDTPW | WDTCNTCL | WDTIS_7; break;
		case 'p' : pause(); break;
	}
	__bic_SR_register_on_exit (LPM0_bits); // Exit LPM0
}

inline void send(uchar c) {
	while (!(UCA0IFG & UCTXIFG));
	UCA0TXBUF = c;
}

inline void send(const char* p) { // better interrupt driven
  while (*p) send(*p++);
}

void sendInt(uint i) {
	send(i & 0xFF);
	send(i >> 8);
}

inline bool serInRdy() {
	return serInPtr != serOutPtr;
}


void sendHex(ulong i, int width = 0) {
  char intStr[8+1];
  char* p = intStr + sizeof(intStr) - 1; // at end
  *p = 0;
  do {
    uint n = i & 0xF;
    *--p = n <= 9 ? n + '0' : n + 'A' - 10;
  	i >>= 4;
    --width;
  } while (i || width > 0);
  send(p);
}

ulong bin2BCD(ulong bin) { // double dabble
  if (!bin) return 0;
  if (bin > 99999999)
  	return 0x99999999;
  ulong bit = 0x4000000; // 8 decimal digits max 99999999 fits in 27 bits
  while (!(bin & bit)) bit >>= 1;

  ulong bcd = 0;
  long carry = 0;
  while (1) {
    bcd <<= 1;
    bcd += carry; // carry to next BCD digit ( 10 + 6 = 16 = LSB of next BCD digit)
    if (bit & bin) bcd |= 1;
    if (!(bit >>= 1)) return bcd;
    carry = ((bcd + 0x33333333) & 0x88888888) >> 1; // carrys: 8s -> 4s
    carry += carry >> 1; // carrys -> 6s
  }
}

void sendDec(long i, const char* tag = ", ", int width = 0) { // 8 digits max
	if (i < 0) {
		send('-');
		i = -i;
	}
	sendHex(bin2BCD(i), width);
	send(tag);
}

#if 0

void sendHex(ullong i, int width = 0) {
  char intStr[16+1];
  char* p = intStr + sizeof(intStr) - 1; // at end
  *p = 0;
  do {
    uint n = i & 0xF;
    *--p = n <= 9 ? n + '0' : n + 'A' - 10;
  	i >>= 4;
    --width;
  } while (i || width > 0);
  send(p);
}

ullong bin2BCD(ullong bin) { // double dabble
  if (!bin) return 0;
  if (bin > 9999999999999999) return 0x9999999999999999;
  ullong bit = 0x20000000000000; // 16 decimal digits max fits in 54 bits
  while (!(bin & bit)) bit >>= 1;

  ullong bcd = 0;
  ullong carry = 0;
  while (1) {
    bcd <<= 1;
    bcd += carry; // carry to next BCD digit ( 10 + 6 = 16 = LSB of next BCD digit)
    if (bit & bin) bcd |= 1;
    if (!(bit >>= 1)) return bcd;
    carry = ((bcd + 0x3333333333333333) & 0x8888888888888888) >> 1; // carrys: 8s -> 4s
    carry += carry >> 1; // carrys -> 6s
  }
}


void sendDec(ullong i, const char* tag = ", ", int width = 0) { // 8 digits max
	sendHex(bin2BCD(i), width);
	send(tag);
}
#endif


void initPins() {
	// pull low vs. TTL signals in
	PAOUT = PBOUT = PCOUT = 0;
	PAREN = PBREN = PCREN = 0xFFFF;
	PJREN = 0xFF;

	P3SEL = RxD | TxD;
	P5SEL = XIN | XOUT;

	PMAPKEYID = PMAPKEY;
	PMAPCTL = PMAPRECFG;
	P4MAP7 = PM_TB0CCR1A;
}


void identify() {
	send("\nPPS 5328 "__DATE__" "__TIME__"\n");
	if (UCSCTL7_L & 0xF) { // why XT2 fault? XT2 not used !!!
		if (UCSCTL7_L & 7) {send('0' + (UCSCTL7_L & 0xF)); send(" x!\n");}
		UCSCTL7_L = 0; // clear faults
	  SFRIFG1 = 0; // clear OFIFG _> switches back to XT1 from REFO
	}
}


void setVCore() { // to level 3
  PMMCTL0_H = PMMPW_H; // Open all PMM registers for write access
	for (int level = 1; level <= 3; ++level) {
		PMMIFG = 0;
		SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level; // Set SVS/SVM high side new level
		SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;                    // Set SVM low side to new level
		while ((PMMIFG & SVSMLDLYIFG) == 0); // Wait till SVM is settled
	  PMMCTL0_L = level;
	  if ((PMMIFG & SVMLIFG))
      while ((PMMIFG & SVMLVLRIFG) == 0); // wait for core to reach low level
  }
	PMMCTL0_H = 0;
}

void stableDCO() {
  while (1) {
  	uint stable = 1024; // a few ms
  	int lastUCSCTL0 = UCSCTL0;
  	while (abs((int)UCSCTL0 - lastUCSCTL0) <= 8 * 4) // ~ 0.25% MOD taps; stable within 1 %
  		if (!stable--) return;
  }
}


void initClocks() {
	uint year = RTCYEAR, date = RTCDATE, hour = RTCTIM1, minSec = RTCTIM0;  // save time

	RTCCTL01 = RTCBCD | RTCMODE; // BCD from ACLK, resets time

	// restore time
  RTCYEAR = year;
	RTCDATE = date;
	RTCTIM1 = hour;
	RTCTIM0 = minSec;

	UCSCTL4 = SELA_2 | SELS_3 | SELM_2;  // ACLK = REFOCLK; SMCLK from DCO to enable DCO;  MCLK = REFOCLK while stabilizing DCO;
	UCSCTL6 = XT2OFF | XT1DRIVE_3 | XCAP_3; // start XT1 w/ 12 pF load

  setVCore();

  while (UCA0STAT & UCBUSY);
	UCA0CTL1 = UCSWRST;

  __bis_SR_register(SCG0);                 // Disable the FLL control loop

  do {
  	UCSCTL7 = 0;
  	__delay_cycles(10000);  // wait for xtal to start -- could check XT1LFOFFG
  } while (UCSCTL7 & 	XT1LFOFFG);

	UCSCTL3 = SELREF_0;   // Set DCO FLL reference = XT1; // _2 = REFOCLK
	UCSCTL1 = DCORSEL_7;  // for 20 to 60+ MHz DCOHz

	const int FLLPow = 1; // CPUHz > 32768L * 1024 ? 1 : 0;
  const int FLLn = (CPUHz + XtalHz / 2) / XtalHz - 1;   // FLLN - 10 bits
  UCSCTL2 = FLLPow * FLLD_1 | FLLn;

  UCSCTL7_L = 0; // clear faults
	__bic_SR_register(SCG0);                 // Enable the FLL control loop

	stableDCO();
  UCSCTL7_L = 0; // clear faults
  SFRIFG1 = 0; // clear OFIFG

	UCSCTL4 = SELA_0 | SELS_4 | SELM_4;       // ACLK = XT1; SMCLK = MCLK = DCOCLKDIV = DCO >> FLLPow
  UCSCTL5 = DIVS_1;  // SMCLK = CPUHz / 2

	UCA0BRW = (CPUHz / 2 + BaudRate / 2) / BaudRate; // from SMCLK
	UCA0CTL1 = UCSSEL_2;

	UCSCTL6 = XT2OFF | XT1DRIVE_0 | XCAP_2; // XT1  XCAP_1:327685     XCAP_2:  32767.719, 29.24C
}

void advance_ms(int ms) { // before ADC interrupts
	while (ms-- > 0) {
		UCSCTL4 |= SELA_4;  // ACLK from DCOCLKDIV = MCLK
		__delay_cycles(CPUHz / (CPUHz / XtalHz - 1) / 1000); // advance 1ms
		UCSCTL4 &= ~SELA_7;
	}
}

int compileLater;

void useLatest(volatile uchar& rtc, char* compile) {
	uchar compileBCD = (compile[0] & 0xF) << 4 | compile[1] & 0xF;
	if (!compileLater)
		compileLater = (int)compileBCD - (char)rtc;
	if (compileLater > 0)
		rtc = compileBCD;
}

void initRTC() {
  char dateStr[] = __DATE__;
	useLatest(RTCYEARH, dateStr + 7);
  useLatest(RTCYEARL, dateStr + 9);

	dateStr[3] = 0;
	const char Months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
	int month = (strstr(Months, dateStr) - Months) / 3 + 1;
	char monthStr[2] = {(char)(month / 10), (char)(month % 10)};
	useLatest(RTCMON, monthStr);
	useLatest(RTCDAY, dateStr + 4);

	char timeStr[] = __TIME__;
	useLatest(RTCHOUR, timeStr);
	useLatest(RTCMIN, timeStr + 3);
  useLatest(RTCSEC, timeStr + 6);
	advance_ms(compileLater > 0 ? 11000 : 400); // advance by ~compile/load time or boot ACLK off time

	// only using hour alarm match
  RTCAMIN = 0;
  RTCADOWDAY = 0;

	// Use 'A' command:  Hz to pin 20
  // PPM = (f / 2064384 - 1) * 1000000
  const int OscErrPPM = -9; // negative = xtal too slow
	const signed char slowDown = OscErrPPM / (OscErrPPM > 0 ? 2 : 4); // steps: -2.035 ppm; +4.069 ppm
	RTCCTL23 = slowDown;
}

void sendTime(const char* sep = ", ", bool date = false) {
	if (date) {sendHex(RTCMON);  send('/'); sendHex(RTCDAY); send('/'); sendHex(RTCYEARL); send(' ');}
	if (RTCHOUR < 10) send('0');
	sendHex(RTCHOUR); send(':'); sendHex(RTCMIN, 2); send(':'); sendHex(RTCSEC,2); send(sep);
}


void initADC(){
	ADC12CTL0 &= ~ADC12ENC;
	while (REFCTL0 & REFGENBUSY);
	REFCTL0 = REFMSTR | REFOUT | REFON;  // 1.5V

	// (256 + 13) / 1.8432 MHz = 146 us ~ 6852 samples/sec
	// (1024 + 13) / 1.8432 MHz = 563 us ~ 1777 samples/sec
	ADC12CTL0 = ADC12SHT0_15 | ADC12MSC | ADC12ON;  // > 100 us temperature sensor settle
	ADC12CTL1 = ADC12SHP | ADC12DIV_7 | ADC12SSEL_3 | ADC12CONSEQ_2;  // want 1 to 2.4 MHz; SMCLK / 8 = 1.8432 MHz
	ADC12CTL2 = ADC12RES_2;  // 12 bit conversion

	ADC12MCTL0 = ADC12EOS | ADC12SREF_1 | ADC12INCH_10;  // temperature

	ADC12CTL0 |= ADC12ENC | ADC12SC;
}


volatile long iTempADC;
volatile int  iSamples;

void sendTemp() {
	long tempADC;
  int samples;
	__bic_SR_register(GIE);
	tempADC = iTempADC;
	samples = iSamples;
	iTempADC = 0;
	iSamples = 0;
	__bis_SR_register(GIE);

  static uint CAL_ADC_T30_1V5 = *(uint*)0x1A1A;
  static uint CAL_ADC_T85_1V5 = *(uint*)0x1A1C;
  static uint CAL_DELTA_55 = CAL_ADC_T85_1V5 - CAL_ADC_T30_1V5;
  int temp100ths = (tempADC - (long)CAL_ADC_T30_1V5 * samples) * (85 - 30) * 100 / CAL_DELTA_55 / samples + 30 * 100;

  sendDec(temp100ths/100, "."); sendDec(temp100ths % 100, ", ", 2);
}


const uint watchdogMargin = 1;  // TB0CCR0 interrupt jitter between setting/checking TB0CCR0
const uint PPS_WDT_Ticks = 32768 + watchdogMargin;

const int NumOsc = 4;
long nomHz[NumOsc];

volatile long secs[NumOsc] = {-1, -1, -1, -1};
volatile int ppsEdge;

volatile bool missed1PPS;

volatile ullong initial[NumOsc];

volatile union {
	ullong ullongv;  // 48 bits incremented -- 1000 hrs of 55 MHz
  struct {
    uint LSW;
    ulong MSW;  // or ullong for > 1000 hrs
  };
} counts[NumOsc];

bool oscIntrpt(volatile uint* TIV, uint oscIdx, uint TCCR1) {  // returns sample ready
	static ulong ovflCount[NumOsc]; // or ullong for > 1000 hrs

	 __bic_SR_register(GIE);  // prevent PPS intrpt during ovfl service: can delay/lose ovfl count
  switch (*TIV + *TIV) {
		case 0xE : // overflow only - lowest priority -> can interrupt before/after edge capture
		  ++ovflCount[oscIdx];
		case 0 :
		  __bis_SR_register(GIE);
		  return false;

  	case 2 :   // just edge capture
  		counts[oscIdx].MSW = ovflCount[oscIdx];
  		break;

		case 2 + 0xE : // near simultaneous edge and ovfl
		  // 4 simultaneous 1 PPS edge interrupts -> delayed service can get overflows
		  // delay > 32768 xtalClocks -> lose an overflow    TODO: or less -- happens often
		  if ((int)TCCR1 >= 0)  // edge occurred at/after overflow
		  	counts[oscIdx].MSW = ++ovflCount[oscIdx];
		  else
		    counts[oscIdx].MSW = ovflCount[oscIdx]++;
			break;
	}

  __bis_SR_register(GIE);

  // 1 PPS edge
	counts[oscIdx].LSW = TCCR1;
	if (++secs[oscIdx] <= 0)
	 	initial[oscIdx] = counts[oscIdx].ullongv;

	ppsEdge |= 1 << oscIdx;
	return true;
}

// interrupts from highest to lowest priority:

#if 0
#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR_HOOK(void) {  // 1PPS watchdog
  TB0CCR0 = TB0R + PPS_WDT_Ticks;  // next watchdog to keep rough time when no satellites in view
	for (uint osc = 0; osc < NumOsc; ++osc)
		++secs[osc];
	//send('.'); // missed 1PPS
}
#endif

#pragma vector=TIMER0_B1_VECTOR
__interrupt void TIMER0_B1_ISR_HOOK(void) {
  if (oscIntrpt(&TB0IV, 0, TB0CCR1)) // each read resets the highest pending interrupt flag
	  __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR_HOOK(void) {
	if (oscIntrpt(&TA0IV, 1, TA0CCR1))
		__bic_SR_register_on_exit(LPM0_bits);
}

#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR_HOOK(void) {
	if (oscIntrpt(&TA1IV, 2, TA1CCR1))
		__bic_SR_register_on_exit(LPM0_bits);
}

#pragma vector=TIMER2_A1_VECTOR
__interrupt void TIMER2_A1_ISR_HOOK(void) { // socketed xtal
	if (oscIntrpt(&TA2IV, 3, TA2CCR1))
	  __bic_SR_register_on_exit(LPM0_bits);
}


void init1PPS() {
  P1SEL = XtalA0 | OnePPSA0 | XtalA1;
  P2SEL = OnePPSA1 | XtalA2 | OnePPSA2;
  P4SEL = OnePPSB;

  TA0CTL = TA1CTL = TA2CTL = TASSEL_0 | MC_2 | TACLR | TAIE; // TxxCLK pin
  TA0CCTL1 = TA1CCTL1 = TA2CCTL1 = TB0CCTL1 = CM_1 | SCS | CAP | CCIE; // PPS rising edge

  TB0CTL = TASSEL_1 | MC_2 | TACLR | TAIE; // ACLK
  TB0CCR0 = TB0R + PPS_WDT_Ticks;
  // TB0CCTL0 = CCIE;  // 1PPS watchdog
}

long reportSecs = 3;

bool setNomHz(int osc) {
	const long xtalHz[] = {32768,
		9830400, 10137600, 13824000, 18432000,// baud, some K1114
		9523810, 14318182, // / 3 * 7
	 25090000,
	 25175000, 45818176, 0};

	// 32768 =                        2^15

	// baud xtals
	// 9.8304  = 38400 * 16 * 16 =    2^17 * 3 *   5^2
	// 18.432 =                       2^14 * 3^2 * 5^3
	// 10.1376 = 38400 * 8 * 3 * 11 = 2^12 * 3^2 * 5^2 * 11
	// 13.824  = 57600 * 16 * 15 =    2^12 * 3^3 * 5^3
	// common:                        2^12 *       5^2 = 102400


	// 9.5238  = 200M / 3 / 7 =       2^9  *       5^8 / 3 / 7
	// 14.31818 =                     2^5  * 3^2 * 5^7 * 7 / 11  =  NTSC
	// 25.175 = 31468.75 * 800 =      2^3  *       5^5 * 19 * 53 VGA pixel
	// common:                        2^3  *       5^5 = 25000 (after * 3 * 7 * 11)

	// 25.090 =                     	2^4  *       5^4 * 13 * 193
	// 45.818176 =                    2^6  * 715909

	const int K1100tol = 10000; // 1 / 0.01%
	const int K1114tol = 2000;  // 1 / 0.05%

	long Hz = (counts[osc].ullongv - initial[osc]) / secs[osc];  // measured

	if (Hz < 10000) return false;

	int f = 0;
	if (labs(Hz - (Hz + 50000) / 100000 * 100000) <= Hz / K1100tol)   // within 1kHz of N * 100kHz
		Hz = (Hz + 50000) / 100000 * 100000;
	//else if (labs(Hz - (Hz + 102400 / 2) / 102400 * 102400) <= Hz / K1100tol)  // pass only ~1% of values but still some wrong
  //	Hz = (Hz + 102400 / 2) / 102400 * 102400;  // baud xtals
	else { // check list of remaining frequencies
		do {
			if (labs(Hz - xtalHz[f]) < Hz / K1114tol) {
				Hz = xtalHz[f];
				break;
			}
		} while (xtalHz[++f]);

		if (!xtalHz[f]) { // non-standard Hz
			static long lastNonStdHz;
			if (Hz > 1000000 && labs(Hz - lastNonStdHz) < Hz / K1100tol) // only if stable after socket startup
				 send("Non-std:");
			else {
				lastNonStdHz = Hz;
				nomHz[osc] = 0;
				secs[osc] = -1;
				return false;
			}
		}
	}

	reportSecs = 3;
	nomHz[osc] = Hz;
	return true;
}


void reportMHz() {
	sendTime();
	sendTemp();

	bool newXtal = false;
	for (uint osc = 0; osc < NumOsc; ++osc) {
		if (nomHz[osc] <= 0) {
			if (setNomHz(osc)) {
			  newXtal = true;
			  int decimals = 6;
			  long Hz = nomHz[osc] % 1000000;
			  while (decimals && Hz % 10 == 0) {
			  	--decimals;
          Hz /= 10;
			  }
			  sendDec(nomHz[osc] / 1000000, ".");
			  sendDec(Hz, ", ", decimals);
			}
		}
	}

	if (!newXtal)	for (uint osc = 0; osc < NumOsc; ++osc) {
		// First few PPM error reports differ from later:
		//  32KHz: truncation: 1 count = 30 ppm / 2s = 15 ppm
		//  others: short-term frequency instability?, Vcc dip?

		if (nomHz[osc]) {
			ullong xtalCounts = counts[osc].ullongv - initial[osc];
			int nomSecs = (xtalCounts + nomHz[osc] / 2) / nomHz[osc];  // 10 PPM -> off after 14 hrs
			if (nomSecs) {
				const long resolution = 1000000000; // PPB
				long ppRes = ((llong)xtalCounts - (llong)nomHz[osc] * nomSecs) * resolution / nomSecs / nomHz[osc];
				sendDec((long)(ppRes / (resolution / 1000000)), "."); // PPM
				sendDec((long)(abs(ppRes) % (resolution / 1000000)), ", ", 3);  // digits beyond PPM decimal
				continue;
			}
		}
	  send("off, ");
	}
  send('\n');
}

const uint measSecs = 1000;  // 17 minutes

void chk1PPS() {
	iTempADC += ADC12MEM0;
	++iSamples;

	if (secs[0] <= 1) return;

	if (!(secs[0] % reportSecs) || secs[0] >= measSecs) {
    reportMHz();
    if (secs[0] >= measSecs) {
       secs[0] = secs[1] = secs[2] = secs[3] = -1; // new measurements
       reportSecs = measSecs;
    } else reportSecs <<= 1;
	}

  if (ppsEdge == 7) { // all but ZIF xtal clock -- why happening?
		secs[3] = -1;
		nomHz[3] = 0;
  }
	ppsEdge = 0;  // or one at a time
}


void chkSerCmd() {
	if (!serInRdy()) return;  // stay in loop

	char cmdChar = *serOutPtr;
	if (++serOutPtr >= SerBufEnd)
		serOutPtr = serBuf;

	static bool enteringDigits;
	static long param;
	if (cmdChar >='0' && cmdChar <= '9') { // isnum
	  if (!enteringDigits) {
	  	enteringDigits = true;
	  	param = 0;
	  }
	  param += param << 2; // * 5
	  param <<= 1;         // * 2 -> * 10
		param += cmdChar & 0xF;
		return;
	}
  enteringDigits = false;

	static bool negative;
	if (cmdChar >= 'a') cmdChar &= 0x5F; // toUpper
	switch (cmdChar) {
		case '/' :
		case ':' : return;  // ignore for date and time entry
		case '-' : negative = true;	return;
		case '+' : negative = false; return;

		case '\r': send("\r\n"); break;
		case 'I' : identify(); break;;
		case 'F' : send("Ctrl-F8 to Free Run\n"); pause(); break;
		case ' ':
		case 'R' : reportMHz(); break;

		case 'T' : // Time
			if (param) { // set time
				if (negative) {  // set date  [2022]1231
					ulong dBCD = bin2BCD(-param);
					if (dBCD & 0xFFFF0000)
						RTCYEAR = dBCD >> 16;
					RTCDATE = dBCD;
				} else {
 				  ulong tBCD = bin2BCD(param); // 235959
			  	RTCHOUR = tBCD >> 16;
				  RTCTIM0 = tBCD; // Minutes:Secs
				}
			}
			sendTime("\n", true);
			param = 0;
			break;

		case 'Z':
			secs[0] = secs[1] = secs[2] = secs[3] = -1;
		case 'X' :
			nomHz[3] = 0;
			secs[3] = -1;
			reportSecs = 3;
		  break;
	}
	negative = false;
}


void main() {
 	WDTCTL = WDTPW | WDTHOLD;

 	initPins();
	initClocks();
	initRTC();

  identify();

	UCA0IE |= UCRXIE;

  init1PPS();
  initADC();  // for temperature

  __bis_SR_register(GIE);

  while (1) {
  	__bis_SR_register(LPM0_bits); // wait for 1 PPS or keystroke
  	chkSerCmd();
  	chk1PPS();
	}
}
