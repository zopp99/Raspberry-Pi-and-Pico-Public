import machine 
from utime import sleep
from utime import ticks_add,ticks_ms,ticks_diff
import sys
from machine import Pin, Timer
import network
import _thread
from lib.umqtt.robust import MQTTClient
from MySecrets import NETWORKSSID, NETWORKPASSWORD

mqtt_message_none = 0
mqtt_message_forcebutton = 1
mqtt_message_sendstats = 2

def interrupt_handler(pin):
    global interrupt_source
    global button
    sleep(.1)   # debounce routine
    if not button.value():
        interrupt_source = 1
        #print("++PIR Interrupt")
    else:
        interrupt_source = 2
        #print("++Button Interrupt")
    return()    

def MQTT_msg_rcvd(topic,msg):
    global mqttmessage_received
    global alarm_is_enabled
    if (msg == b'turn_on' and not alarm_is_enabled) or (msg == b'turn_off'and alarm_is_enabled):
        mqttmessage_received = mqtt_message_forcebutton
    if (msg == b'request_for_status'):
        mqttmessage_received = mqtt_message_sendstats
    print("received topic =",str(topic))
    print("received message =",str(msg))
    
def lan_connect():
    led.on()
    timerblink = Timer()
    timerblink.init(period=100,mode=Timer.PERIODIC,callback=lambda t:led.toggle())
    last_check_msg=ticks_ms()
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(NETWORKSSID,NETWORKPASSWORD)
    while not wlan.isconnected():
        B=ticks_diff(ticks_ms(),last_check_msg)
        if (B > 10000):machine.reset()
    print('network connected in uSecs: ' + str(ticks_diff(ticks_ms(),last_check_msg)))
    timerblink.deinit()
    sleep(2)
    led.off()


def mqtt_thread_subscribe():
    mqtt_server = 'broker.hivemq.com'
    client_id = 'client-002'
    client = MQTTClient(client_id, mqtt_server, port=1883, keepalive=600)
    print('attempting subscribe in thread')
    try:
        client.connect()
    except OSError as error:
        print("OSError was thrown at mqtt_connect_subscribe: ", error)
        reconnect()
    client.set_callback(MQTT_msg_rcvd)
    #print('Subscribe Connected to %s MQTT Broker'%(mqtt_server))
    client.subscribe(topic_sub)
        # Handle MQTT messages
    while True:
        client.wait_msg()



def mqtt_connect_publish():
    topic_successful_mqtt_connection = b'Connected to MQTT broker'
    mqtt_server = 'broker.hivemq.com'
    client_id = 'client-001'
    client = MQTTClient(client_id, mqtt_server, port=1883, keepalive=600)
    try:
        client.connect()
    except OSError as error:
        print("OSError was thrown at mqtt_connect_publish: ", error)
        reconnect()
    client.publish(topic_pub, topic_successful_mqtt_connection)
    return client

def reconnect():
    print('Failed to connect to the MQTT Broker. Restarting...')
    sleep(5)
    machine.reset()

def main(rtn_code):
    global alarm_is_enabled
    global interrupt_source
    global mqttmessage_received
    global button
    global topic_pub
    global topic_sub
    global led
    global lock_mqtt_connect
    alarm_trigger_count = 0
    alarm_is_enabled = False
    mqttmessage_received = mqtt_message_none
    interrupt_source = 0
    led = Pin("LED", Pin.OUT)   # on board LED for Pico W
    led.off()
    button = Pin(16,Pin.IN,Pin.PULL_DOWN)
    buzzer = machine.Pin(27,machine.Pin.OUT)
    sensor_interrupt = machine.Pin(28, machine.Pin.IN, machine.Pin.PULL_DOWN)
    sensor_interrupt.irq(trigger=machine.Pin.IRQ_RISING, handler=interrupt_handler)

    lan_connect()
    led.off()
    
    topic_pub = "house/dogalarm"                # Outbound msg
    topic_siren_activated = b'Movement Detected - Siren Activated'
    topic_dog_alarm_stats = b'Times siren has been activated: '
    topic_monitoring_started = b'Confirmed: DogProximityAlarm has begun monitoring'
    topic_monitoring_ended = b'Confirmed: DogProximityAlarm has ended monitoring'
    topic_sub = "house/dogalarm_command"       # inbound msg



    # Create and start the MQTT thread to read published messages
    sleep(5)
    mqtt_thread = _thread.start_new_thread(mqtt_thread_subscribe, ())
    sleep(15)
    client = mqtt_connect_publish()
    sleep(5)



 

    timerbuzzer=Timer()
    last_check_msg=ticks_ms()

    while True:
        #print('looping',end='')
        #print('\b\b\b\b\b\b\b',end='')
        #print('       ',end='')
        #print('\b\b\b\b\b\b\b',end='')
        if interrupt_source == 0:
            if ticks_diff(ticks_ms(),last_check_msg)>1:
                #client.check_msg()
                last_check_msg=ticks_ms()
            if mqttmessage_received == mqtt_message_sendstats:
                mqttmessage_received = mqtt_message_none
                c = topic_dog_alarm_stats + str(alarm_trigger_count)
                print('sending stats')
                client.publish(topic_pub,c)

        if interrupt_source == 1:                     #PIR caused the interrupt
            interrupt_source = 0
            sleep(.2)
            if sensor_interrupt.value() and alarm_is_enabled == True:
                buzzer.on()
                timerbuzzer=Timer(period=5000, mode=Timer.ONE_SHOT, callback=lambda t:buzzer.off())
                timerbuzzer.init
                alarm_trigger_count += 1
                print('Publish alarm')
                client.publish(topic_pub, topic_siren_activated)
                

        if interrupt_source == 2 or mqttmessage_received == mqtt_message_forcebutton:    # Button caused the interrupt
            interrupt_source = 0
            timerbuzzer.deinit
            buzzer.off()
            alarm_in_process = False
            sleep(1)
            if  button.value() or mqttmessage_received == mqtt_message_forcebutton:
                mqttmessage_received = mqtt_message_none
                buzzer.on()
                sleep(.1)
                buzzer.off()
                if alarm_is_enabled == False:
                    client.publish(topic_pub, topic_monitoring_started)
                    alarm_trigger_count = int(0)
                    for i in range(10):
                        led.off()
                        sleep(0.5)
                        led.on()
                        sleep(0.5)

                    alarm_is_enabled = True
                else:
                    client.publish(topic_pub, topic_monitoring_ended)
                    led.off()  
                    alarm_is_enabled = False
     
            else:
                for i in range(alarm_trigger_count):
                    led.off()
                    sleep(0.25)
                    led.on()
                    sleep(0.25)        

        if mqttmessage_received == mqtt_message_sendstats:
            print('Publish sendstats')
            client.publish(topic_pub, topic_dog_alarm_stats+str(alarm_trigger_count))



#if __name__ == "__main__":  #__name__ is not set in pico since first module is actually boot.py
sys.exit(main(sys.argv))

