#include <Arduino.h>
// #include <HardwareSerial.h>
#include <SPI.h>
#include <VarioSettings.h>
#include <I2Cdev.h>
#include <ms5611.h>
#include <vertaccel.h>
#include <EEPROM.h>
#include <LightInvensense.h>
#include <kalmanvert.h>
#include <beeper.h>
#include <toneAC.h>
#include <avr/pgmspace.h>
#include <varioscreen.h>
#include <digit.h>
#include <SdCard.h>
#include <LightFat16.h>
// #include <SerialNmea.h>
#include <NmeaParser.h>
#include <LxnavSentence.h>
#include <LK8Sentence.h>
#include <IGCSentence.h>
#include <FirmwareUpdater.h>


/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*!!            !!! WARNING  !!!              !!*/
/*!! Before building check :                  !!*/
/*!! libraries/VarioSettings/VarioSettings.h  !!*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/


/*******************/
/* General objects */
/*******************/
#define VARIOMETER_POWER_ON_DELAY 2000

#define VARIOMETER_STATE_INITIAL 0
#define VARIOMETER_STATE_DATE_RECORDED 1
#define VARIOMETER_STATE_CALIBRATED 2
#define VARIOMETER_STATE_FLIGHT_STARTED 3

#ifdef HAVE_GPS
uint8_t variometerState = VARIOMETER_STATE_INITIAL;
#else
uint8_t variometerState = VARIOMETER_STATE_CALIBRATED;
#endif //HAVE_GPS

/***************/
/* IMU objects */
/***************/
#ifdef HAVE_ACCELEROMETER
#ifdef IMU_CALIBRATION_IN_EEPROM
VertaccelSettings vertaccelSettings = Vertaccel::readEEPROMSettings();
#else //!IMU_CALIBRATION_IN_EEPROM
const VertaccelSettings vertaccelSettings = {
  IMU_GYRO_CAL_BIAS
  ,{ IMU_ACCEL_CAL_BIAS, IMU_ACCEL_CAL_SCALE }
#ifdef AK89xx_SECONDARY
  ,{ IMU_MAG_CAL_BIAS, IMU_MAG_CAL_PROJ_SCALE }
#endif //AK89xx_SECONDARY
};
#endif //!IMU_CALIBRATION_IN_EEPROM

Vertaccel vertaccel(vertaccelSettings);
#endif //HAVE_ACCELEROMETER


/*****************/
/* screen objets */
/*****************/
#ifdef HAVE_SCREEN

unsigned long lastLowFreqUpdate = 0;

#ifdef HAVE_GPS
#define VARIOSCREEN_ALTI_ANCHOR_X 52
#define VARIOSCREEN_ALTI_ANCHOR_Y 0
#define VARIOSCREEN_VARIO_ANCHOR_X 52
#define VARIOSCREEN_VARIO_ANCHOR_Y 2
#define VARIOSCREEN_TIME_ANCHOR_X 5
#define VARIOSCREEN_TIME_ANCHOR_Y 0
#define VARIOSCREEN_ELAPSED_TIME_ANCHOR_X 5
#define VARIOSCREEN_ELAPSED_TIME_ANCHOR_Y 3
#define VARIOSCREEN_SPEED_ANCHOR_X 22
#define VARIOSCREEN_SPEED_ANCHOR_Y 4
#define VARIOSCREEN_GR_ANCHOR_X 72
#define VARIOSCREEN_GR_ANCHOR_Y 4
#define RATIO_MAX_VALUE 30.0
#define RATIO_MIN_SPEED 10.0
#else
#define VARIOSCREEN_ALTI_ANCHOR_X 52
#define VARIOSCREEN_ALTI_ANCHOR_Y 1
#define VARIOSCREEN_VARIO_ANCHOR_X 52
#define VARIOSCREEN_VARIO_ANCHOR_Y 3
#endif //HAVE_GPS

#define VARIOSCREEN_BAT_ANCHOR_X 68
#define VARIOSCREEN_BAT_ANCHOR_Y 1
#define VARIOSCREEN_SAT_ANCHOR_X 68
#define VARIOSCREEN_SAT_ANCHOR_Y 0

#define VARIOSCREEN_MUTE_ANCHOR_X 2
#define VARIOSCREEN_MUTE_ANCHOR_Y 0

VarioScreen screen(VARIOSCREEN_DC_PIN,VARIOSCREEN_CS_PIN,VARIOSCREEN_RST_PIN);
MSUnit msunit(screen, VARIOSCREEN_VARIO_ANCHOR_X, VARIOSCREEN_VARIO_ANCHOR_Y);
MUnit munit(screen, VARIOSCREEN_ALTI_ANCHOR_X, VARIOSCREEN_ALTI_ANCHOR_Y);
ScreenDigit altiDigit(screen, VARIOSCREEN_ALTI_ANCHOR_X, VARIOSCREEN_ALTI_ANCHOR_Y, 0, false);
ScreenDigit varioDigit(screen, VARIOSCREEN_VARIO_ANCHOR_X, VARIOSCREEN_VARIO_ANCHOR_Y, 1, true);
#ifdef HAVE_GPS
ScreenDigit speedDigit(screen, VARIOSCREEN_SPEED_ANCHOR_X, VARIOSCREEN_SPEED_ANCHOR_Y, 0, false);
ScreenDigit ratioDigit(screen, VARIOSCREEN_GR_ANCHOR_X, VARIOSCREEN_GR_ANCHOR_Y, 1, false);
KMHUnit kmhunit(screen, VARIOSCREEN_SPEED_ANCHOR_X, VARIOSCREEN_SPEED_ANCHOR_Y);
GRUnit grunit(screen, VARIOSCREEN_GR_ANCHOR_X, VARIOSCREEN_GR_ANCHOR_Y);
SATLevel satLevel(screen, VARIOSCREEN_SAT_ANCHOR_X, VARIOSCREEN_SAT_ANCHOR_Y);
ScreenTime screenTime(screen, VARIOSCREEN_TIME_ANCHOR_X, VARIOSCREEN_TIME_ANCHOR_Y);
ScreenElapsedTime screenElapsedTime(screen, VARIOSCREEN_ELAPSED_TIME_ANCHOR_X, VARIOSCREEN_ELAPSED_TIME_ANCHOR_Y);
#endif //HAVE_GPS
#ifdef HAVE_ACCELEROMETER
ScreenMuteIndicator muteIndicator(screen, VARIOSCREEN_MUTE_ANCHOR_X, VARIOSCREEN_MUTE_ANCHOR_Y);
#endif //HAVE_ACCELEROMETER
#ifdef HAVE_VOLTAGE_DIVISOR
BATLevel batLevel(screen, VARIOSCREEN_BAT_ANCHOR_X, VARIOSCREEN_BAT_ANCHOR_Y, VOLTAGE_DIVISOR_VALUE, VOLTAGE_DIVISOR_REF_VOLTAGE);
#endif //HAVE_VOLTAGE_DIVISOR


ScreenSchedulerObject displayList[] = { {&msunit, 0}, {&munit, 0}, {&altiDigit, 0}, {&varioDigit, 0}
#ifdef HAVE_GPS
                                       ,{&kmhunit, 0}, {&grunit, 0}, {&speedDigit, 0}, {&ratioDigit, 0}, {&satLevel, 0}, {&screenTime, 1}, {&screenElapsedTime, 1}
#endif //HAVE_GPS
#ifdef HAVE_ACCELEROMETER
                                       ,{&muteIndicator, 0}
#endif //HAVE_ACCELEROMETER
#ifdef HAVE_VOLTAGE_DIVISOR
                                       , {&batLevel, 0}
#endif //HAVE_VOLTAGE_DIVISOR
};

#ifdef HAVE_GPS
ScreenScheduler varioScreen(screen, displayList, sizeof(displayList)/sizeof(ScreenSchedulerObject), 0, 1);
#else
ScreenScheduler varioScreen(screen, displayList, sizeof(displayList)/sizeof(ScreenSchedulerObject), 0, 0);
#endif //HAVE_GPS

#endif //HAVE_SCREEN

/**********************/
/* alti/vario objects */
/**********************/
#define POSITION_MEASURE_STANDARD_DEVIATION 0.1
#ifdef HAVE_ACCELEROMETER
#define ACCELERATION_MEASURE_STANDARD_DEVIATION 0.3
#else
#define ACCELERATION_MEASURE_STANDARD_DEVIATION 0.6
#endif //HAVE_ACCELEROMETER

kalmanvert kalmanvert;

#ifdef HAVE_SPEAKER
beeper beeper(VARIOMETER_SINKING_THRESHOLD, VARIOMETER_CLIMBING_THRESHOLD, VARIOMETER_NEAR_CLIMBING_SENSITIVITY, VARIOMETER_BEEP_VOLUME);
#endif

/***************/
/* gps objects */
/***************/
#ifdef HAVE_GPS

NmeaParser nmeaParser;

#ifdef HAVE_BLUETOOTH
boolean lastSentence = false;
#endif //HAVE_BLUETOOTH

unsigned long RMCSentenceTimestamp; //for the speed filter
double RMCSentenceCurrentAlti; //for the speed filter
unsigned long speedFilterTimestamps[VARIOMETER_SPEED_FILTER_SIZE];
double speedFilterSpeedValues[VARIOMETER_SPEED_FILTER_SIZE];
double speedFilterAltiValues[VARIOMETER_SPEED_FILTER_SIZE];
int8_t speedFilterPos = 0;

#ifdef HAVE_SDCARD
lightfat16 file(SDCARD_CS_PIN);
IGCHeader header;
IGCSentence igc;

#define SDCARD_STATE_INITIAL 0
#define SDCARD_STATE_INITIALIZED 1
#define SDCARD_STATE_READY 2
#define SDCARD_STATE_ERROR -1
int8_t sdcardState = SDCARD_STATE_INITIAL;

#endif //HAVE_SDCARD

#endif //HAVE_GPS

/*********************/
/* bluetooth objects */
/*********************/
#ifdef HAVE_BLUETOOTH
#if defined(VARIOMETER_SENT_LXNAV_SENTENCE)
LxnavSentence bluetoothNMEA;
#elif defined(VARIOMETER_SENT_LK8000_SENTENCE)
LK8Sentence bluetoothNMEA;
#else
#error No bluetooth sentence type specified !
#endif

#ifndef HAVE_GPS
unsigned long lastVarioSentenceTimestamp = 0;
#endif // !HAVE_GPS
#endif //HAVE_BLUETOOTH

#if defined(HAVE_ACCELEROMETER) && defined(HAVE_SPEAKER)
/* tap callback : mute/unmute beeper */
void beeperTapCallback(unsigned char direction, unsigned char count) {

  static bool muted = false;
  muted = !muted;
  toneACMute(muted);
#ifdef HAVE_SCREEN
  muteIndicator.setMuteState(muted);
#endif //HAVE_SCREEN
}
#endif //defined(HAVE_ACCELEROMETER) && defined(HAVE_SPEAKER)


/*-----------------*/
/*      SETUP      */
/*-----------------*/
void setup() {
   Serial.begin(9600);
   Serial.println("Serial started");

  /*****************************/
  /* wait for devices power on */
  /*****************************/
  delay(VARIOMETER_POWER_ON_DELAY);

  /**********************/
  /* init accelerometer */
  /**********************/
  Fastwire::setup(FASTWIRE_SPEED, 0);
#ifdef HAVE_ACCELEROMETER
  vertaccel.init();
  if( firmwareUpdateCond() ) {
   firmwareUpdate();
  }
  fastMPUSetTapCallback(beeperTapCallback);
#endif //HAVE_ACCELEROMETER

  /************/
  /* init SPI */
  /************/

  /* set all SPI CS lines before talking to devices */
#if defined(HAVE_SDCARD) && defined(HAVE_GPS)
  file.enableSPI();
#endif //defined(HAVE_SDCARD) && defined(HAVE_GPS)

#ifdef HAVE_SCREEN
  screen.enableSPI();
#endif //HAVE_SCREEN

  /****************/
  /* init SD Card */
  /****************/
#if defined(HAVE_SDCARD) && defined(HAVE_GPS)
  if( file.init() >= 0 ) {
    sdcardState = SDCARD_STATE_INITIALIZED;  //useless to set error
  }
#endif //defined(HAVE_SDCARD) && defined(HAVE_GPS)

  /***************/
  /* init screen */
  /***************/
#ifdef HAVE_SCREEN
  screen.begin(VARIOSCREEN_CONTRAST);
#endif //HAVE_SCREEN

  /******************/
  /* init barometer */
  /******************/
  ms5611_init();

  /******************/
  /* get first data */
  /******************/

  /* wait for first alti and acceleration */
  while( ! (ms5611_dataReady()
#ifdef HAVE_ACCELEROMETER
            && vertaccel.dataReady()
#endif //HAVE_ACCELEROMETER
            ) ) {
  }

  /* get first data */
  ms5611_updateData();

  /* init kalman filter */
  kalmanvert.init(ms5611_getAltitude(),
#ifdef HAVE_ACCELEROMETER
                  vertaccel.getValue(),
#else
                  0.0,
#endif
                  POSITION_MEASURE_STANDARD_DEVIATION,
                  ACCELERATION_MEASURE_STANDARD_DEVIATION,
                  millis());

}

#if defined(HAVE_SDCARD) && defined(HAVE_GPS)
void createSDCardTrackFile(void);
#endif //defined(HAVE_SDCARD) && defined(HAVE_GPS)
void enableflightStartComponents(void);

/*----------------*/
/*      LOOP      */
/*----------------*/
void loop() {

  /*****************************/
  /* compute vertical velocity */
  /*****************************/
#ifdef HAVE_ACCELEROMETER
  if( ms5611_dataReady() && vertaccel.dataReady() ) {
    ms5611_updateData();

    double barAlt = ms5611_getAltitude();
    double vertAcc = vertaccel.getValue();

    // Monitor Kalman input with accel
    Serial.print("Kalman input(with vertaccel): ");
    Serial.print("altitude: ");
    Serial.print(barAlt);
    Serial.print("acceleration: ");
    Serial.print(vertAcc);
    Serial.println("");

    kalmanvert.update( barAlt,
                       vertAcc,
                       millis() );
#else
  if( ms5611_dataReady() ) {
    ms5611_updateData();

    // Monitor Kalman input with accel
    // Serial.print("Kalman input(no vertaccel): ");
    // Serial.print("altitude: ");
    // Serial.print(barAlt);
    // Serial.print("acceleration: ");
    // Serial.print(0.0);
    // Serial.println("");

    kalmanvert.update( ms5611_getAltitude(),
                       0.0,
                       millis() );
#endif //HAVE_ACCELEROMETER

    /* set beeper */
#ifdef HAVE_SPEAKER
    beeper.setVelocity( kalmanvert.getVelocity() );
#endif //HAVE_SPEAKER

    /* set screen */
#ifdef HAVE_SCREEN
    altiDigit.setValue(kalmanvert.getCalibratedPosition());
    varioDigit.setValue(kalmanvert.getVelocity() );
#endif //HAVE_SCREEN


  }

  /*****************/
  /* update beeper */
  /*****************/
#ifdef HAVE_SPEAKER
  beeper.update();
#endif //HAVE_SPEAKER


    /***************************/
    /* update variometer state */
    /*    (after parsing)      */
    /***************************/
    if( variometerState < VARIOMETER_STATE_FLIGHT_STARTED ) {

      /* if initial state check if date is recorded  */
      if( variometerState == VARIOMETER_STATE_INITIAL ) {
        if( nmeaParser.haveDate() ) {
          variometerState = VARIOMETER_STATE_DATE_RECORDED;
        }
      }

      /* check if we need to calibrate the altimeter */
      else if( variometerState == VARIOMETER_STATE_DATE_RECORDED ) {

        /* we need a good quality value */
        if( nmeaParser.haveNewAltiValue() && nmeaParser.precision < VARIOMETER_GPS_ALTI_CALIBRATION_PRECISION_THRESHOLD ) {

          /* calibrate */
          double gpsAlti = nmeaParser.getAlti();
          kalmanvert.calibratePosition(gpsAlti);
          variometerState = VARIOMETER_STATE_CALIBRATED;
        }
      }

      /* else check if the flight have started */
      else {  //variometerState == VARIOMETER_STATE_CALIBRATED

        /* check flight start condition */
        if( (millis() > FLIGHT_START_MIN_TIMESTAMP) &&
            (kalmanvert.getVelocity() < FLIGHT_START_VARIO_LOW_THRESHOLD || kalmanvert.getVelocity() > FLIGHT_START_VARIO_HIGH_THRESHOLD) &&
            (nmeaParser.getSpeed() > FLIGHT_START_MIN_SPEED) ) {
          variometerState = VARIOMETER_STATE_FLIGHT_STARTED;
          enableflightStartComponents();
        }
      }
    }

  /* if no GPS, we can't calibrate, and we have juste to check flight start */
#ifndef HAVE_GPS
  if( variometerState == VARIOMETER_STATE_CALIBRATED ) { //already calibrated at start
    if( (millis() > FLIGHT_START_MIN_TIMESTAMP) &&
        (kalmanvert.getVelocity() < FLIGHT_START_VARIO_LOW_THRESHOLD || kalmanvert.getVelocity() > FLIGHT_START_VARIO_HIGH_THRESHOLD) ) {
      variometerState = VARIOMETER_STATE_FLIGHT_STARTED;
      enableflightStartComponents();
    }
  }
#endif // !HAVE_GPS

  /**********************************/
  /* update low freq screen objects */
  /**********************************/
#ifdef HAVE_SCREEN
  unsigned long lowFreqDuration = millis() - lastLowFreqUpdate;
  if( varioScreen.getPage() == 0 ) {
    if( lowFreqDuration > VARIOMETER_BASE_PAGE_DURATION ) {
#ifdef HAVE_GPS //no multipage without GPS
      varioScreen.nextPage();
    }
  } else { //page == 1
    if( lowFreqDuration > VARIOMETER_BASE_PAGE_DURATION + VARIOMETER_ALTERNATE_PAGE_DURATION ) {
#endif //HAVE_GPS multipage support
      lastLowFreqUpdate = millis();

#ifdef HAVE_GPS
      /* set time */
      screenTime.setTime( nmeaParser.time );
      screenTime.correctTimeZone( VARIOMETER_TIME_ZONE );
      screenElapsedTime.setCurrentTime( screenTime.getTime() );

      /* update satelite count */
      satLevel.setSatelliteCount( nmeaParser.satelliteCount );
#endif //HAVE_GPS

#ifdef HAVE_VOLTAGE_DIVISOR
      /* update battery level */
      batLevel.setVoltage( analogRead(VOLTAGE_DIVISOR_PIN) );
#endif //HAVE_VOLTAGE_DIVISOR

#ifdef HAVE_GPS //no multipage without GPS
      varioScreen.nextPage();
#endif //HAVE_GPS multipage support
    }
  }


  /*****************/
  /* update screen */
  /*****************/
#ifdef HAVE_GPS
  /* when getting speed from gps, display speed and ratio */
  if ( nmeaParser.haveNewSpeedValue() ) {

    /* get new values */
    unsigned long baseTime = speedFilterTimestamps[speedFilterPos];
    unsigned long deltaTime = RMCSentenceTimestamp; //delta computed later
    speedFilterTimestamps[speedFilterPos] = RMCSentenceTimestamp;

    double deltaAlti = speedFilterAltiValues[speedFilterPos]; //computed later
    speedFilterAltiValues[speedFilterPos] = RMCSentenceCurrentAlti;

    double currentSpeed = nmeaParser.getSpeed();
    speedFilterSpeedValues[speedFilterPos] = currentSpeed;

    speedFilterPos++;
    if( speedFilterPos >= VARIOMETER_SPEED_FILTER_SIZE )
      speedFilterPos = 0;

    /* compute deltas */
    deltaAlti -= RMCSentenceCurrentAlti;
    deltaTime -= baseTime;

    /* compute mean distance */
    double meanDistance = 0;
    int step = 0;
    while( step < VARIOMETER_SPEED_FILTER_SIZE ) {

      /* compute distance */
      unsigned long currentTime = speedFilterTimestamps[speedFilterPos];
      meanDistance += speedFilterSpeedValues[speedFilterPos] * (double)(currentTime - baseTime);
      baseTime = currentTime;

      /* next */
      speedFilterPos++;
      if( speedFilterPos >= VARIOMETER_SPEED_FILTER_SIZE )
        speedFilterPos = 0;
      step++;
    }

    /* compute glide ratio */
    double ratio = (meanDistance/3600.0)/deltaAlti;

    /* display speed and ratio */
    speedDigit.setValue( currentSpeed );
    if( currentSpeed >= RATIO_MIN_SPEED && ratio >= 0.0 && ratio < RATIO_MAX_VALUE ) {
      ratioDigit.setValue(ratio);
    } else {
      ratioDigit.setValue(0.0);
    }
  }
#endif //HAVE_GPS

  /* screen update */
  varioScreen.displayStep();

#endif //HAVE_SCREEN
}



#if defined(HAVE_SDCARD) && defined(HAVE_GPS)
void createSDCardTrackFile(void) {
  /* start the sdcard record */
  if( sdcardState == SDCARD_STATE_INITIALIZED ) {

    /* some cards doesn't like delays between init and write, so reinit */
    file.init();

    /* build date : convert from DDMMYY to YYMMDD */
    uint8_t dateChar[8]; //two bytes are used for incrementing number on filename
    uint8_t* dateCharP = dateChar;
    uint32_t date = nmeaParser.date;
    for(uint8_t i=0; i<3; i++) {
      uint8_t num = ((uint8_t)(date%100));
      dateCharP[0] = (num/10) + '0';
      dateCharP[1] = (num%10) + '0';
      dateCharP += 2;
      date /= 100;
    }

    /* create file */
    if( file.begin((char*)dateChar, 8) >= 0 ) {
      sdcardState = SDCARD_STATE_READY;

      /* write the header */
      int16_t datePos = header.begin();
      if( datePos >= 0 ) {
        while( datePos ) {
        file.write(header.get());
          datePos--;
        }

        /* write date : DDMMYY */
        uint8_t* dateCharP = &dateChar[4];
        for(int i=0; i<3; i++) {
          file.write(dateCharP[0]);
          file.write(dateCharP[1]);
          header.get();
          header.get();
          dateCharP -= 2;
        }

        while( header.available() ) {
          file.write(header.get());
        }
      }
    } else {
      sdcardState = SDCARD_STATE_ERROR; //avoid retry
    }
  }
}
#endif //defined(HAVE_SDCARD) && defined(HAVE_GPS)



void enableflightStartComponents(void) {
  /* set base time */
#if defined(HAVE_SCREEN) && defined(HAVE_GPS)
  screenElapsedTime.setBaseTime( screenTime.getTime() );
#endif //defined(HAVE_SCREEN) && defined(HAVE_GPS)

  /* enable near climbing */
#ifdef HAVE_SPEAKER
#ifdef VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM
  beeper.setGlidingAlarmState(true);
#endif
#ifdef VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP
  beeper.setGlidingBeepState(true);
#endif
#endif //HAVE_SPEAKER

#if defined(HAVE_SDCARD) && defined(HAVE_GPS) && defined(VARIOMETER_RECORD_WHEN_FLIGHT_START)
  createSDCardTrackFile();
#endif // defined(HAVE_SDCARD) && defined(VARIOMETER_RECORD_WHEN_FLIGHT_START)
}

