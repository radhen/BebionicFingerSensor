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
    TARG_FORCE = 0.55

    ########### SAVING DATA ###################
    now = rospy.get_rostime()
    sec = now.secs
    nsec = now.nsecs
    ################## PCF ######################
    b_1, b_2, b_3, b_4, b_5 = msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4]
    ir_1, ir_2, ir_3, ir_4, ir_5 = msg.data[5], msg.data[6], msg.data[7], msg.data[8], msg.data[9]
    nn_1, nn_2, nn_3, nn_4, nn_5 = msg.data[10], msg.data[11], msg.data[12], msg.data[13], msg.data[14]
    args[8] = np.append(args[8], np.array([[sec, nsec, b_1, b_2, b_3, b_4, b_5, ir_1, ir_2, ir_3, ir_4, ir_5, nn_1, nn_2, nn_3, nn_4, nn_5]]), axis=0)

    if args[6] == 0:
        print "enter once"
        global min_1
        min_1 = msg.data[10]
        global min_2
        min_2 = msg.data[11]
        global min_3
        min_3 = msg.data[12]
        global min_4
        min_4 = msg.data[13]
        global min_5
        min_5 = msg.data[13]
        args[6] = 1

    #keep track of running min
    if msg.data[10] < min_1: min_1 = msg.data[10]
    if msg.data[11] < min_2: min_2 = msg.data[11]
    if msg.data[12] < min_3: min_3 = msg.data[12]
    if msg.data[13] < min_4: min_4 = msg.data[13]
    if msg.data[14] < min_5: min_5 = msg.data[14]

    # normalize nn ouput for each sensor
    f_1 = (msg.data[10] - min_1) / 0.8
    f_2 = (msg.data[11] - min_2) / 0.8
    f_3 = (msg.data[12] - min_3) / 0.8
    f_4 = (msg.data[13] - min_4) / 0.8
    f_5 = (msg.data[14] - min_5) / 0.8

    # print f_1, f_2, f_3, f_4, f_5

    e_curr_1 = TARG_FORCE - f_1
    e_curr_2 = TARG_FORCE - f_2
    e_curr_3 = TARG_FORCE - f_3
    e_curr_4 = TARG_FORCE - f_4
    e_curr_5 = TARG_FORCE - f_5

    # print e_curr_1, e_curr_2, e_curr_3, e_curr_4

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

    u_t_1 = args[2] * e_curr_1 #+ args[3] * args[0][0] + args[4] * diff_e_1
    u_t_2 = args[2] * e_curr_2 #+ args[3] * args[0][1] + args[4] * diff_e_2
    u_t_3 = args[2] * e_curr_3 #+ args[3] * args[0][2] + args[4] * diff_e_3
    u_t_4 = args[2] * e_curr_4 #+ args[3] * args[0][3] + args[4] * diff_e_4
    u_t_5 = args[2] * e_curr_4 #+ args[3] * args[0][4] + args[4] * diff_e_5

    # rescale from 0-3 to 0-255 (PWM)

    # print u_t_1, u_t_2, u_t_3, u_t_4, u_t_5

    pwm_1 = int(((u_t_1) / (1.0)) * 1.0)
    pwm_2 = int(((u_t_2) / (1.0)) * 1.0)
    pwm_3 = int(((u_t_3) / (1.0)) * 1.0)
    pwm_4 = int(((u_t_4) / (1.0)) * 1.0)
    pwm_5 = int(((u_t_5) / (1.0)) * 1.0)

    # pwm = np.clip(pwm,0,250)

    # print pwm_1, '\t', e_curr_1, '\t', pwm_2, '\t', e_curr_2, '\t', pwm_3, '\t', e_curr_3, '\t', pwm_4, '\t', e_curr_4

    # y_predict = [e_curr, u_t, pwm]
    # msg = Float32MultiArray(MultiArrayLayout([MultiArrayDimension('pid_output', 2, 1)], 1), y_predict)
    # args[5].publish(msg)

    e_curr = [e_curr_1, e_curr_2, e_curr_3, e_curr_4, e_curr_5]
    pwm = [pwm_1, pwm_2, pwm_3, pwm_4, pwm_5]

    # time.sleep(5)

    if not addList:
        pid_sub.unregister()
        path = '/home/radhen/Documents/bebionic_expData/'
        np.savetxt(path + '/banana_{}.txt'.format(1), args[8])
        print "Exiting from subscriber and saving data"
    else:
        for i in addList:
            if -0.25 < e_curr[int(i) - 1] < 0.1:
                args[9].apply_breaks(str(i))
                print "Applying breaks {}".format(i)
                addList.remove(i)
                print addList
            else:
                args[9].fully_close(str(i), abs(pwm[int(i) - 1]))


if __name__ == "__main__":

    rospy.init_node('bebionic_hand_control')

    mf = MotorFunctions()

    # addList = ['3','4']
    # addList = ['6']


    ############ Testing poistion control thru PID control ###############

    # for i in addList: mf.set_position_count(str(i), 1000)
    # for i in addList: mf.set_target_position(str(i), 10000)
    # for i in addList: mf.set_pid_gains(str(i))
    # for i in addList: mf.enable_pid(str(i))
    # rospy.sleep(1)
    # for i in addList: mf.apply_breaks(str(i))
    #
    # for i in addList: mf.set_position_count(str(i), 10000)
    # for i in addList: mf.set_target_position(str(i), 1000)
    # for i in addList: mf.set_pid_gains(str(i))
    # for i in addList: mf.enable_pid(str(i))
    # rospy.sleep(1)
    # for i in addList: mf.apply_breaks(str(i))


    ############ Testing force control thru PID control ###############

    # pcf_data = np.zeros((1, 17))
    # pid_pub = rospy.Publisher("/pid_output", Float32MultiArray, queue_size=1)
    # 
    # sum_e_1 = 0
    # sum_e_2 = 0
    # sum_e_3 = 0
    # sum_e_4 = 0
    # sum_e_5 = 0
    # sum_e_list = [sum_e_1,sum_e_2,sum_e_3,sum_e_4,sum_e_5]
    # 
    # e_last_1 = 0
    # e_last_2 = 0
    # e_last_3 = 0
    # e_last_4 = 0
    # e_last_5 = 0
    # e_last_list = [e_last_1, e_last_2, e_last_3, e_last_4, e_last_1]
    # 
    # kp = 70.0
    # ki = 0.05
    # kd = 0.08
    # count = 0
    # 
    # sum_e_arr_1 = np.zeros(25)
    # sum_e_arr_2 = np.zeros(25)
    # sum_e_arr_3 = np.zeros(25)
    # sum_e_arr_4 = np.zeros(25)
    # sum_e_arr_5 = np.zeros(25)
    # sum_e_arr_list = [sum_e_arr_1,sum_e_arr_2,sum_e_arr_3,sum_e_arr_4,sum_e_arr_5]
    # 
    # # pid_sub = rospy.Subscriber("/sensor_values", Float32MultiArray, pid_callback, [sum_e_1,sum_e_2,sum_e_3,sum_e_4,sum_e_5,e_last_1,e_last_2,e_last_3,e_last_4,e_last_5,
    # #                                                                                kp,ki,kd,pid_pub,count,sum_e_arr_1,sum_e_arr_2,sum_e_arr_3,sum_e_arr_4,pcf_data])
    # 
    # pid_sub = rospy.Subscriber("/sensor_values", Float32MultiArray, pid_callback,
    #                            [sum_e_list, e_last_list, kp, ki, kd, pid_pub, count, sum_e_arr_list, pcf_data, mf])
    # 
    # rospy.spin()


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

