import RPi.GPIO as GPIO
import time
class ultra:
    EchoPin=99
    TrigPin=99
    #Ignore the warning message

    #Set pin mode
    def __init__(self,e,t):



        #Define the pins of the ultrasonic module
        self.EchoPin = e
        self.TrigPin = t
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.EchoPin,GPIO.IN)
        GPIO.setup(self.TrigPin,GPIO.OUT)
        self.dist = 0;


    @property
    def distance(self):
        return self.dist

    @distance.setter
    def distance(self, value):
        if val == -1:
            val = 200
        self.dist = min(max(0, value), 200)

    def calcDistance(self):

        GPIO.output(self.TrigPin,GPIO.LOW)
        time.sleep(0.000002)
        GPIO.output(self.TrigPin,GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(self.TrigPin,GPIO.LOW)

        start_time = time.time()

        while not GPIO.input(self.EchoPin):
            t4 = time.time()
            if (t4 - start_time) > 0.03 :
                return 200
            
        t1 = time.time()
        while GPIO.input(self.EchoPin):
            t5 = time.time()
            if(t5 - t1) > 0.03 :
                return 200

        t2 = time.time()
        self.distance = ((t2 - t1)* 340 / 2) * 100
        return self.distance

    def Distance_test(this):

        num = 0
        ultrasonic = []
        while num < 5:
                distance = this.calcDistance()
                #print("distance is %f"%(distance) )
                while int(distance) == -1 :
                    distance = this.calcDistance()
                    #print("Tdistance is %f"%(distance) )
                while (int(distance) >= 500 or int(distance) == 0) :
                    distance = this.calcDistance()
                    #print("Edistance is %f"%(distance) )
                ultrasonic.append(distance)
                num = num + 1
                #time.sleep(0.01)
        #print ('ultrasonic')
        distance = (ultrasonic[1] + ultrasonic[2] + ultrasonic[3])/3
        #print("distance is %f"%(distance) ) 
        return distance

if __name__ == "__main__":
    front = ultra(10, 26)
    right = ultra(22, 16)
    left  = ultra(9, 11)
    
    while True:
        print(f"Front: {front.calcDistance()}\nRight: {right.calcDistance()}\nLeft: {left.calcDistance()}")
        time.sleep(0.1)

