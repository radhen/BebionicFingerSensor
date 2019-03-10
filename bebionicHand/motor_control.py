#!/usr/bin/env python
from motor_functions import MotorFunctions
import rospy



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

    ########### SAVING DATA ###################
    now = rospy.get_rostime()
    sec = now.secs
    nsec = now.nsecs
    ################## PCF ######################
    b_1, b_2, b_3, b_4, b_5 = msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4]
    ir_1, ir_2, ir_3, ir_4, ir_5 = msg.data[5], msg.data[6], msg.data[7], msg.data[8], msg.data[9]
    nn_1, nn_2, nn_3, nn_4, nn_5 = msg.data[10], msg.data[11], msg.data[12], msg.data[13], msg.data[14], msg.data[15]
    args[17] = np.append(args[17], np.array([[sec, nsec, b_1, b_2, b_3, b_4, b_5, ir_1, ir_2, ir_3, ir_4, ir_5, nn_1, nn_2, nn_3, nn_4, nn_5]]), axis=0)

    if args[12] == 0:
        print "enter once"
        global min_1
        min_1 = msg.data[8]
        global min_2
        min_2 = msg.data[9]
        global min_3
        min_3 = msg.data[10]
        global min_4
        min_4 = msg.data[11]
        args[12] = 1

    #keep track of running min
    if msg.data[8] < min_1: min_1 = msg.data[8]
    if msg.data[9] < min_2: min_2 = msg.data[9]
    if msg.data[10] < min_3: min_3 = msg.data[10]
    if msg.data[11] < min_4: min_4 = msg.data[11]

    # normalize nn ouput for each sensor
    f_1 = (msg.data[8] - min_1) / 0.8
    f_2 = (msg.data[9] - min_2) / 0.8
    f_3 = (msg.data[10] - min_3) / 0.8
    f_4 = (msg.data[11] - min_4) / 0.8

    # print f_1, f_2, f_3, f_4

    e_curr_1 = TARG_FORCE - f_1
    e_curr_2 = TARG_FORCE - f_2
    e_curr_3 = TARG_FORCE - f_3
    e_curr_4 = TARG_FORCE - f_4

    # print e_curr_1, e_curr_2, e_curr_3, e_curr_4

    push(sum_e_arr_1, [e_curr_1])
    args[0] = sum(sum_e_arr_1)
    push(sum_e_arr_2, [e_curr_2])
    args[1] = sum(sum_e_arr_2)
    push(sum_e_arr_3, [e_curr_3])
    args[2] = sum(sum_e_arr_3)
    push(sum_e_arr_4, [e_curr_4])
    args[3] = sum(sum_e_arr_4)

    diff_e_1 = e_curr_1 - args[4]
    args[4] = e_curr_1
    diff_e_2 = e_curr_2 - args[5]
    args[5] = e_curr_2
    diff_e_3 = e_curr_3 - args[6]
    args[6] = e_curr_3
    diff_e_4 = e_curr_4 - args[7]
    args[7] = e_curr_4

    u_t_1 = args[8] * e_curr_1 + args[9] * args[0] + args[10] * diff_e_1
    u_t_2 = args[8] * e_curr_2 + args[9] * args[1] + args[10] * diff_e_2
    u_t_3 = args[8] * e_curr_3 + args[9] * args[2] + args[10] * diff_e_3
    u_t_4 = args[8] * e_curr_4 + args[9] * args[3] + args[10] * diff_e_4

    # rescale from 0-3 to 0-255 (PWM)

    # print u_t_1, u_t_2, u_t_3, u_t_4

    pwm_1 = int(((u_t_1) / (1.4)) * 50)
    pwm_2 = int(((u_t_2) / (1.4)) * 50)
    pwm_3 = int(((u_t_3) / (1.4)) * 50)
    pwm_4 = int(((u_t_4) / (1.4)) * 50)

    # pwm = np.clip(pwm,0,250)

    # print pwm_1, '\t', e_curr_1, '\t', pwm_2, '\t', e_curr_2, '\t', pwm_3, '\t', e_curr_3, '\t', pwm_4, '\t', e_curr_4

    # y_predict = [e_curr, u_t, pwm]
    # msg = Float32MultiArray(MultiArrayLayout([MultiArrayDimension('pid_output', 2, 1)], 1), y_predict)
    # args[5].publish(msg)

    e_curr = [e_curr_1, e_curr_2, e_curr_3, e_curr_4]
    pwm = [pwm_1, pwm_2, pwm_3, pwm_4]

    # time.sleep(5)

    if not addList:
        pid_sub.unregister()
        path = '/home/radhen/Documents/bebionic_expData/'
        np.savetxt(path + '/right_{}.txt'.format(1), args[17])
        print "Exiting from subscriber and saving data"
    else:
        for i in addList:
            if -0.25 < e_curr[int(i) - 1] < 0.25:
                apply_breaks(ser, str(i))
                print "Applying breaks {}".format(i)
                addList.remove(i)
            else:
                fully_close(ser, str(i), abs(pwm[int(i) - 1]))


if __name__ == "__main__":

    rospy.init_node('bebionic_hand_control')

    mf = MotorFunctions()

    addList = ['4','5']
    # addList = ['5']


    ############ Testing poistion control thru PID control ###############

    # for i in addList: mf.set_position_count(str(i), 1000)
    # for i in addList: mf.set_target_position(str(i), 4000)
    # for i in addList: mf.set_pid_gains(str(i))
    # for i in addList: mf.enable_pid(str(i))
    # rospy.sleep(1)
    # for i in addList: mf.apply_breaks(str(i))


    ############ Testing force control thru PID control ###############

    # pcf_data = np.zeros((1, 14))
    # pid_pub = rospy.Publisher("/pid_output", Float32MultiArray, queue_size=1)
    # sum_e_1 = 0
    # sum_e_2 = 0
    # sum_e_3 = 0
    # sum_e_4 = 0
    # e_last_1 = 0
    # e_last_2 = 0
    # e_last_3 = 0
    # e_last_4 = 0
    # kp = 1.5
    # ki = 0.05
    # kd = 0.08
    # count = 0
    # sum_e_arr_1 = np.zeros(25)
    # sum_e_arr_2 = np.zeros(25)
    # sum_e_arr_3 = np.zeros(25)
    # sum_e_arr_4 = np.zeros(25)
    #
    # pid_sub = rospy.Subscriber("/sensor_values", Float32MultiArray, pid_callback, [sum_e_1,sum_e_2,sum_e_3,sum_e_4, e_last_1,e_last_2,e_last_3,e_last_4,
    #                                                                                kp,ki,kd,pid_pub,count,sum_e_arr_1,sum_e_arr_2,sum_e_arr_3,sum_e_arr_4,pcf_data])
    #
    # rospy.spin()

