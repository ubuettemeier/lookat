/**
 * @file    raspi.hpp
 * @author  Ulrich Buettemeier
 * @date    2021-12-19
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <iostream>
#include <thread>
#include <wiringPi.h>

using namespace std;

class raspi {
public:
    /*******************************************
     * @brief Construct a new raspi object
     */
    raspi() {
        // if (wiringPiSetup() == -1) {
        if (wiringPiSetupGpio() == -1) {    // Instalise wiringPi with Broadcom GPIO pins
            cout << "can not setup wiringPi\n";
            return;
        }

        thread t(run);      // thread einrichten.
        t.detach();         // thread frei laufend
    }    

    /*******************************************
     * @brief Destroy the raspi object
     */
    ~raspi() {
        ende = 1;
        while (ende != 2);
    }  
      
    /******************************************
     * @brief   eine sinnlose Funktion
     */
    static void switch_port(int state = -1)
    {
        static int out_state = 1;
        
        // cout << "NOW:" << out_state << endl;
        if (state != -1)
            out_state = state;

        digitalWrite (14, out_state);
        out_state = (out_state == 0) ? 1 : 0;
    }
    
    /******************************************
     * @brief Thread
     */
    static void run() 
    {
        pinMode (14, OUTPUT);

        while (!ende) {                        
            raspi::switch_port();
            usleep (10000);
        }

        raspi::switch_port(0);
        cout << "thread beendet\n";
        ende = 2;
    }

private:    
    static int ende;
};  // end off class raspi

int raspi::ende = 0;    // used for kill thread
raspi rpi;              // Autostart
