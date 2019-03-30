#!/usr/bin/env python
from motor_functions import MotorFunctions
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension



def push(x, y):
    push_len = len(y)
    assert len(x) >= push_len
    x[:-push_len] = x[push_len:]
    x[-push_len:] = y
    return x


def pid_callback(msg, args):
    # print (msg.data[0])
    # pass

    emptyList = []
    TARG_DIS = 0.06

    ########### SAVING DATA ###################
    now = rospy.get_rostime()
    sec = now.secs
    nsec = now.nsecs
    ################## PCF ######################
    ir_1, ir_2, ir_3, ir_4, ir_5 = msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4]
    # ir_1, ir_2, ir_3, ir_4, ir_5 = msg.data[5], msg.data[6], msg.data[7], msg.data[8], msg.data[9]
    # nn_1, nn_2, nn_3, nn_4, nn_5 = msg.data[10], msg.data[11], msg.data[12], msg.data[13], msg.data[14]
    args[8] = np.append(args[8], np.array([[sec, nsec, ir_4, msg.data[8]]]), axis=0)

    if args[6] == 0:
        print "enter once"
        global min_1
        min_1 = msg.data[0]
        global min_2
        min_2 = msg.data[1]
        global min_3
        min_3 = msg.data[2]
        global min_4
        min_4 = msg.data[3]
        global min_5
        min_5 = msg.data[4]
        args[6] = 1

    #keep track of running min
    if msg.data[0] < min_1: min_1 = msg.data[0]
    if msg.data[1] < min_2: min_2 = msg.data[1]
    if msg.data[2] < min_3: min_3 = msg.data[2]
    if msg.data[3] < min_4: min_4 = msg.data[3]
    if msg.data[4] < min_5: min_5 = msg.data[4]

    # normalize nn ouput for each sensor
    f_1 = (msg.data[0] - min_1)
    f_3 = (msg.data[2] - min_3)
    f_2 = (msg.data[1] - min_2)
    f_4 = (msg.data[3] - min_4)
    f_5 = (msg.data[4] - min_5)

    # print f_1, '\t', f_2, '\t', f_3, '\t', f_4, '\t', f_5, '\n'

    e_curr_1 = TARG_DIS - f_1
    e_curr_2 = TARG_DIS - f_2
    e_curr_3 = TARG_DIS - f_3
    e_curr_4 = TARG_DIS - f_4
    e_curr_5 = TARG_DIS - f_5

    # print e_curr_1, '\t', e_curr_2, '\t', e_curr_3, '\t', e_curr_4, '\t', e_curr_5, '\n'
    # print e_curr_1

    push(args[7][0], [e_curr_1])
    args[0][0] = sum(args[7][0])
    push(args[7][1], [e_curr_2])
    args[0][1] = sum(args[7][1])
    push(args[7][2], [e_curr_3])
    args[0][2] = sum(args[7][2])
    push(args[7][3], [e_curr_4])
    args[0][3] = sum(args[7][3])
    push(args[7][4], [e_curr_5])
    args[0][4] = sum(args[7][4])

    diff_e_1 = e_curr_1 - args[1][0]
    args[1][0] = e_curr_1
    diff_e_2 = e_curr_2 - args[1][1]
    args[1][1] = e_curr_2
    diff_e_3 = e_curr_3 - args[1][2]
    args[1][2] = e_curr_3
    diff_e_4 = e_curr_4 - args[1][3]
    args[1][3] = e_curr_4
    diff_e_5 = e_curr_5 - args[1][4]
    args[1][4] = e_curr_5

    u_t_1 = args[2][0] * e_curr_1 + args[3] * args[0][0] + args[4] * diff_e_1
    u_t_2 = args[2][1] * e_curr_2 + args[3] * args[0][1] + args[4] * diff_e_2
    u_t_3 = args[2][2] * e_curr_3 + args[3] * args[0][2] + args[4] * diff_e_3
    u_t_4 = args[2][3] * e_curr_4 + args[3] * args[0][3] + args[4] * diff_e_4
    u_t_5 = args[2][4] * e_curr_4 + args[3] * args[0][4] + args[4] * diff_e_5

    # rescale from 0-3 to 0-255 (PWM)

    # print u_t_1, '\t', u_t_2, '\t', u_t_3, '\t', u_t_4, '\t', u_t_5, '\n'

    pwm_1 = int(((u_t_1) / (1.0)) * 1.0)
    pwm_2 = int(((u_t_2) / (1.0)) * 1.0)
    pwm_3 = int(((u_t_3) / (1.0)) * 1.0)
    pwm_4 = int(((u_t_4) / (1.0)) * 1.0)
    pwm_5 = int(((u_t_5) / (1.0)) * 1.0)

    # pwm = np.clip(pwm,0,250)

    # print pwm_1, '\t', pwm_2, '\t', pwm_3, '\t', pwm_4, '\t', pwm_5, '\n'

    # y_predict = [1/np.square(msg.data[0]), 1/np.square(msg.data[1]), 1/np.square(msg.data[2]), 1/np.square(msg.data[3]), 1/np.square(msg.data[4])]
    # msg = Float32MultiArray(MultiArrayLayout([MultiArrayDimension('pid_output', 5, 1)], 1), y_predict)
    # args[5].publish(msg)

    e_curr = [e_curr_1, e_curr_2, e_curr_3, e_curr_4, e_curr_5]
    pwm = [pwm_1, pwm_2, pwm_3, pwm_4, pwm_5]

    # time.sleep(5)

    # if not addList:
    #     pid_sub.unregister()
    #     path = '/home/radhen/Documents/bebionic_expData/'
    #     np.savetxt(path + '/banana_{}.txt'.format(1), args[8])
    #     print "Exiting from subscriber and saving data"
    # else:
    #     for i in addList:
    #         if e_curr[int(i) - 1] < 0.005:
    #             args[9].apply_breaks(str(i))
    #             print "Applying breaks {}".format(i)
    #             addList.remove(i)
    #             print addList
    #         if e_curr[int(i)-1] > 0.005:
    #             args[9].fully_close(str(i), abs(pwm[int(i) - 1]))
            # if e_curr[int(i)-1] < -0.01:
            #     print ("overshooot!")

    ## PWM TEST ##
    # for i in addList:
    #     if msg.data[8] > 1000:
    #         args[9].fully_close(str(i), 34)
    #     else:
    #         print "Applying breaks {}".format(i)
    #         args[9].apply_breaks(str(i))
    #         pid_sub.unregister()
    #         path = '/home/radhen/Documents/bebionic_expData/'
    #         np.savetxt(path + '/pwm_34.txt', args[8])
    #         print "Exiting from subscriber and saving data"



if __name__ == "__main__":

    rospy.init_node('bebionic_hand_control')

    mf = MotorFunctions()

    # addList = ['3','4']
    addList = ['4']


    ############ Testing poistion control thru PID control ###############

    for i in addList: mf.set_position_count(str(i), 3000)
    # for i in addList: mf.set_target_position(str(i), 2000)
    # for i in addList: mf.set_pid_gains(str(i))
    # for i in addList: mf.enable_pid(str(i))
    # rospy.sleep(1)
    # for i in addList: mf.apply_breaks(str(i))

    # for i in addList: mf.set_position_count(str(i), 10000)
    # for i in addList: mf.set_target_position(str(i), 1000)
    # for i in addList: mf.set_pid_gains(str(i))
    # for i in addList: mf.enable_pid(str(i))
    # rospy.sleep(1)
    # for i in addList: mf.apply_breaks(str(i))


    ############ Testing force control thru PID control ###############

    pcf_data = np.zeros((1, 4))
    pid_pub = rospy.Publisher("/pid_output", Float32MultiArray, queue_size=1)

    sum_e_1 = 0
    sum_e_2 = 0
    sum_e_3 = 0
    sum_e_4 = 0
    sum_e_5 = 0
    sum_e_list = [sum_e_1,sum_e_2,sum_e_3,sum_e_4,sum_e_5]

    e_last_1 = 0
    e_last_2 = 0
    e_last_3 = 0
    e_last_4 = 0
    e_last_5 = 0
    e_last_list = [e_last_1, e_last_2, e_last_3, e_last_4, e_last_1]

    kp = [300.0, 300.0, 375.0, 500.0, 300.0]
    ki = 10
    kd = 5
    count = 0

    sum_e_arr_1 = np.zeros(25)
    sum_e_arr_2 = np.zeros(25)
    sum_e_arr_3 = np.zeros(25)
    sum_e_arr_4 = np.zeros(25)
    sum_e_arr_5 = np.zeros(25)
    sum_e_arr_list = [sum_e_arr_1,sum_e_arr_2,sum_e_arr_3,sum_e_arr_4,sum_e_arr_5]

    pid_sub = rospy.Subscriber("/sensor_values", Float32MultiArray, pid_callback,
                               [sum_e_list, e_last_list, kp, ki, kd, pid_pub, count, sum_e_arr_list, pcf_data, mf])

    rospy.spin()


    ############ "Manipulate" ###############

    # addList = ['3', '4']
    # addList = ['6']

    # for i in addList: mf.set_position_count(str(i), 2000)
    # for i in addList: mf.set_target_position(str(i), 1000)
    # for i in addList: mf.set_pid_gains(str(i))
    # for i in addList: mf.enable_pid(str(i))
    # rospy.sleep(1)
    # for i in addList: mf.apply_breaks(str(i))

    # d = {0: ['3', 2000, 1750], 1: ['6', 1000, 10000]}
    # for key in d: mf.set_position_count(d[key][0], d[key][1])
    # for key in d: mf.set_target_position(d[key][0], d[key][2])
    # for key in d: mf.set_pid_gains(d[key][0])
    # for key in d: mf.enable_pid(d[key][0])
    # rospy.sleep(1)
    # for key in d: mf.apply_breaks(d[key][0])
    #
    # d = {0: ['3', 1750, 2000], 1: ['6', 10000, 1000]}
    # for key in d: mf.set_position_count(d[key][0], d[key][1])
    # for key in d: mf.set_target_position(d[key][0], d[key][2])
    # for key in d: mf.set_pid_gains(d[key][0])
    # for key in d: mf.enable_pid(d[key][0])
    # rospy.sleep(1)
    # for key in d: mf.apply_breaks(d[key][0])

