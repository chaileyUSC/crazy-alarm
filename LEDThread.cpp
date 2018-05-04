/**
 * Copyright (c) 2017, Autonomous Networks Research Group. All rights reserved.
 * Developed by:
 * Autonomous Networks Research Group (ANRG)
 * University of Southern California
 * http://anrg.usc.edu/
 *
 * Contributors:
 * Jason A. Tran <jasontra@usc.edu>
 * Bhaskar Krishnamachari <bkrishna@usc.edu>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy 
 * of this software and associated documentation files (the "Software"), to deal
 * with the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 * - Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimers.
 * - Redistributions in binary form must reproduce the above copyright notice, 
 *     this list of conditions and the following disclaimers in the 
 *     documentation and/or other materials provided with the distribution.
 * - Neither the names of Autonomous Networks Research Group, nor University of 
 *     Southern California, nor the names of its contributors may be used to 
 *     endorse or promote products derived from this Software without specific 
 *     prior written permission.
 * - A citation to the Autonomous Networks Research Group must be included in 
 *     any publications benefiting from the use of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS WITH 
 * THE SOFTWARE.
 */

/**
 * @file       LEDThread.cpp
 * @brief      Implementation of thread that handles LED requests.
 *
 * @author     Jason Tran <jasontra@usc.edu>
 * @author     Bhaskar Krishnachari <bkrishna@usc.edu>
 */

#include "LEDThread.h"
#include "MQTTmbed.h"
#include "MQTTNetwork.h"

#include "MQTTClient.h"
#include "mbed.h"
#include "m3pi.h"


Mail<MailMsg, LEDTHREAD_MAILBOX_SIZE> LEDMailbox;

static DigitalOut led2(LED2);
AnalogIn ultrasound(p20);


m3pi m3pi(p23, p9, p10);

static const char *topic = "anrg-pi4/robot";

int alarmType = -1; 
int pathType = -1; 
int LEDType = -1;  
int timeDuration = -1; 


void walk(int walkType) //MAKES THE ROBOT WALK IN ITS DESIRED PATH
{
    if (walkType == 0) //NORMAL PATH 
    {
        m3pi.forward(0.5);
        wait(1);
        m3pi.left(1); 
        wait(1); 
        m3pi.forward(0.5);
        wait(1);


        m3pi.stop();  
    }
    else if (walkType == 1) //CRAZY PATH 
    {
        m3pi.forward(3);
        wait(1);
        m3pi.forward(3);
        wait(2);
        m3pi.left(3.6);
        wait(1.4);
        m3pi.right(2.4);
        wait(1.5);
        m3pi.left(1.3);
        wait(2.3); 
        m3pi.backward(1.5);
        wait(2.7); 
        m3pi.stop(); 

    }

}

void go(int walkType)
{
    walk(walkType);
    if (ultrasound*5 < 10)
        m3pi.left(0.5);
}

void playAlarm(int alarmType) //PLAYS A PARTICULAR ALARM FOR THE ROBOT 
{
    if (alarmType == 0) //NORMAL ALARM 
    {
        char* tune = "g32";
        m3pi.playBuzzer(tune); 
    }
    else if (alarmType == 1) //CRAZY ALARM 
    {
        char* tune = "a b c d e d c b";
        m3pi.playBuzzer(tune);
    }
           
}

void LEDPattern(int ledType) //DISPLAYS A PARTICULAR LED PATTERN 
{

    if (ledType == 0) //NORMAL LED PATTERN 
    {  
        m3pi.leds(2);
        wait(0.1); 
        m3pi.leds(4);
        wait(0.1);
        m3pi.leds(8);
        wait(0.1);
        m3pi.leds(16);
        wait(0.1);
        m3pi.leds(32);
        wait(0.1);
        m3pi.leds(64);
        wait(0.1);
        m3pi.leds(128);
        wait(0.1);
        m3pi.leds(256);
        wait(0.1);  
    }
    else if (ledType == 1) //CRAZY LED PATTERN 
    {
        m3pi.leds(10);
        wait(0.1);
        m3pi.leds(112);
        wait(0.01);
        m3pi.leds(124);
        wait(0.01);
    }      
    
    
}
    


void alarm(int alarmType, int pathType, int ledType)
{
    m3pi.printf("ALARM"); 
    LEDPattern(ledType);
    playAlarm(alarmType);
    go(pathType);
}




void LEDThread(void *args) 
{
    MQTT::Client<MQTTNetwork, Countdown> *client = (MQTT::Client<MQTTNetwork, Countdown> *)args;
    MailMsg *msg;
    MQTT::Message message;
    osEvent evt;
    char pub_buf[16];

    while(1) {

        evt = LEDMailbox.get();

        if(evt.status == osEventMail) {
            msg = (MailMsg *)evt.value.p;

            /* the second byte in the message denotes the action type */
            switch (msg->content[1]) {
                case LED_THR_PUBLISH_MSG:
                    printf("LEDThread: received command to publish to topic"
                           "m3pi-mqtt-example/led-thread\n");
                    pub_buf[0] = 'h';
                    pub_buf[1] = 'i';
                    message.qos = MQTT::QOS0;
                    message.retained = false;
                    message.dup = false;
                    message.payload = (void*)pub_buf;
                    message.payloadlen = 2; //MQTTclient.h takes care of adding null char?
                    /* Lock the global MQTT mutex before publishing */
                    mqttMtx.lock();
                    client->publish(topic, message);
                    mqttMtx.unlock();
                    break;
                case LED_ON_ONE_SEC:
                    printf("LEDThread: received message to turn LED2 on for"
                           "one second...\n");
                    led2 = 1;
                    wait(1);
                    led2 = 0;
                    break;
                case LED_BLINK_FAST:
                    printf("LEDThread: received message to blink LED2 fast for"
                           "one second...\n");
                    for(int i = 0; i < 10; i++)
                    {
                        led2 = !led2;
                        wait(0.1);
                    }
                    led2 = 0;
                    break;
                case 'a':
                    alarmType = (int)(msg->content[2]); 
                    m3pi.playBuzzer("c");
                    printf("ALARM TYPE SET");
                    break;
                case 'p':
                    pathType = (int)(msg->content[2]); 
                    m3pi.playBuzzer("d");
                    printf("PATH TYPE SET"); 
                    break;
                case 'l':
                    LEDType = (int)(msg->content[2]);
                    m3pi.playBuzzer("e");
                    printf("LED TYPE SET");
                    break;
                case 't':
                    timeDuration = (int)(msg->content[2]);
                    if (alarmType != -1 && pathType != -1 && LEDType != -1 && timeDuration != -1)
                    {
                        printf("ABOTTA GO OFF!");
                        wait(0.1*timeDuration);
                        printf("GOING OFF!");
                        while (1)
                        {
                            if (alarmType == 0)
                                m3pi.playBuzzer("c");
                            else if (alarmType == 1)
                                m3pi.playBuzzer("a b c d e d c b");
                            if (LEDType == 0)
                            {
                                m3pi.leds(2);
                                wait(0.1); 
                                m3pi.leds(4);
                                wait(0.1);
                                m3pi.leds(8);
                                wait(0.1);
                                m3pi.leds(16);
                                wait(0.1);
                                m3pi.leds(32);
                                wait(0.1);
                                m3pi.leds(64);
                                wait(0.1);
                                m3pi.leds(128);
                                wait(0.1);
                                m3pi.leds(256);
                                wait(0.1); 
                            }
                            else if (LEDType == 1)
                            {
                                m3pi.leds(10);
                                wait(0.1);
                                m3pi.leds(112);
                                wait(0.01);
                                m3pi.leds(124);
                                wait(0.01);        
                            }
                            if (pathType == 0)
                            {
                                 m3pi.forward(0.5);
                                wait(1);
                                m3pi.left(1); 
                                wait(1); 
                                m3pi.forward(0.5);
                                wait(1);


                                m3pi.stop();     
                            }
                            else if (pathType == 1)
                            {
                                m3pi.forward(3);
                                wait(1);
                                m3pi.forward(3);
                                wait(2);
                                m3pi.left(3.6);
                                wait(1.4);
                                m3pi.right(2.4);
                                wait(1.5);
                                m3pi.left(1.3);
                                wait(2.3); 
                                m3pi.backward(1.5);
                                wait(2.7); 
                                m3pi.stop(); 
                            }


                        }
 
                                               
                    }


                    /**
                    if (alarmType != -1 && pathType != -1 && LEDType != -1)
                    {
                        printf("TIMER STARTED \n"); 
                        printf("timeDuration: " + timeDuration);
                        for (int i = 0; i < timeDuration; i++)
                        {
                            printf("ONESEC \n"); 
                            wait(1); 
                        }
                        while (button.read() != -1)
                            alarm(alarmType, pathType, LEDType);
                    }
                    break; 
                default:
                    printf("LEDThread: invalid message\n");
                    **/
                    break;

            }            


            LEDMailbox.free(msg);
        }
    } /* while */

    /* this should never be reached */
}

Mail<MailMsg, LEDTHREAD_MAILBOX_SIZE> *getLEDThreadMailbox() 
{
    return &LEDMailbox;
}


