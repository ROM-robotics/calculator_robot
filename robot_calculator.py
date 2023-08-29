#!/usr/bin/env python3
import sys
from math import pi
def main():
    
        print("===================================================")
        print(" 1) Calculate RobotVelocity to wheel velocity(rad/s) ")
        print(" 2) Ziegler Nichols PID Gains ")
        print(" 3) Calculate actual rpm to actual RobotVelocity")
        print("===================================================")
        choice = int(input("Enter : "))


        if(choice == 1):
            robotVel2WheelRPM()
        elif(choice == 2):
            ZNpidControl()
        elif(choice == 3):
            act_rpm_to_act_robot_vel()
        else:
            sys.exit()

def act_rpm_to_act_robot_vel():
    Vr_ = float(input(" Right wheel              ( rpm )  : "))
    Vl_ = float(input(" Left wheel               ( rpm )  : "))
    L_ = float(input(" Distance between 2 wheels( cm  )  : "))
    d_ = float(input(" Wheel Diameter           ( cm  )  : "))
    dt = float(input(" Time difference                   : "))

    L = 1.0 * cm2Meter(L_)
    d = 1.0 * cm2Meter(d_)

    # rpm to meter per second
    # Vr = Vr * (rev/1 min) x (1 min/60s) x (pi*d/1 rev) = ms
    # Vl = Vl * (rev/1 min) x (1 min/60s) x (pi*d/1 rev) = ms
    Vr = Vr_ * pi * d /60.0
    Vl = Vl_ * pi * d /60.0

    # ave_rpm = Vr + Vl / 2 = rpm
    # ave_rpm = (Vr + Vl) /2.0
    V  = (Vr + Vl) / 2.0

    # Meter per second to Meter (ave_vel to ave_xy)
    # v = s/t ==>  s = vt   = meter
    # ave_xy = ave_vel * dt = meter
    ave_xy = V * dt

    # Meter per second to Meter
    # Vr = Vr * dt = meter , Vl = Vl * dt = meter
    # Vr_ = Vr * dt
    # Vl_ = Vl * dt

    # Angular Velocity (W)
    # W = (Vr - Vl) / L
    W = (Vr - Vl) / L
    print("\n")
    print("               RESULT TABLE")
    print("==============================================")
    print(" INPUT                     | unit |           ")
    print("----------------------------------------------")
    print(" Right wheel RPM           | rpm  |  ", Vr_)
    print(" Left  wheel RPM           | rpm  |  ", Vl_)
    print(" Distance between 2 wheels |  cm  |  ",L_ )
    print(" Wheel Diameter            |  cm  |  ",d_ )
    print(" Delta t                   |   s  |  ", dt)
    print("______________________________________________")
    print(" OUTPUT                                       ")
    print("----------------------------------------------")
    print(" Linear Velocity     =====>|  ms  |  ", V )
    print(" Angular Velocity    =====>|  rs  |  ", W )
    print("==============================================")


#------------------------------------------------------------------------------------


def robotVel2WheelRPM():
    V  = float(input(" Linear Velocity (meter per second): "))
    W  = float(input(" Angular Velocity (rad per second) : "))
    L_ = float(input(" Distance between 2 wheels(m)     : "))
    r_ = float(input(" Wheel Radius        (m)        : "))

    # centimeter to Meter
    #L = L_ * 0.01
    #d = d_ * 0.01
    L = L_ * 1.0
    r = r_ * 1.0

    # 1) Robot Velocity to Left Wheel Right Wheel Velocity
    # linear_velocity = V = (2 * pi * R)/ T
    # angular_velocity= W = (2 * pi) / T
    # V = W R
    # Vl= W(R+L/2) , Vr= W(R-L/2)
    # Vl= WR + WL/2, Vr= WR-WL/2
    Vr = V + (W * L/2.0) * 1.0
    Vl = V - (W * L/2.0) * 1.0

    # 2) wheel linear_velocity to wheel angular_velocity
    # radian per second
    wl = Vl / r 
    wr = Vr / r

    # # 3) radian per second to rev per min
    # wl_ = ( wl * 60.0 ) / ( 2.0 * pi )
    # wr_ = ( wr * 60.0 ) / ( 2.0 * pi )

    # Shortened
    #wl = Vl * 60.0 / ( pi * d )
    #wr = Vr * 60.0 / ( pi * d )
    print("\n")
    print("                 RESULT TABLE")
    print("==============================================")
    print("  INPUT                    | unit |           ")
    print("----------------------------------------------")
    print(" Linear Velocity           |  ms  |  ", V      )
    print(" Angular Velocity          |  rs  |  ", W      )
    print(" Distance between 2 wheels |  cm  |  ",L_      )
    print(" Wheel Radius              |  cm  |  ",r_      )
    print("______________________________________________")
    print("  OUTPUT                                      ")
    print("----------------------------------------------")
    print(" Right wheel   =====>   | rad_per_sec  |  ", wr)
    print(" Left  wheel   =====>   | rad_per_sec  |  ", wl)

    print("==============================================")

def cm2Meter(x):
    return x*0.01

def ZNpidControl():
    KpPrime = float(input("Enter Kp' = "))
    Tc = float(input("Enter Tc in seconds = "))
    Kp = KpPrime*0.6
    Ki = Tc/2
    Kd = Tc/8

    print("\n")
    print("                 RESULT TABLE")
    print("==============================================")
    print("  INPUT                    | unit |           ")
    print("----------------------------------------------")
    print(" Kp Prime                           ", KpPrime)
    print(" Tc                        |  s  |  ", Tc      )
    print("______________________________________________")
    print("  OUTPUT                                      ")
    print("----------------------------------------------")
    print(" Kp                  =====>  ", Kp)
    print(" Ki                  =====>  ", Ki)
    print(" Kd                  =====>  ", Kd)
    print("==============================================")
if __name__ == "__main__":
    main()
