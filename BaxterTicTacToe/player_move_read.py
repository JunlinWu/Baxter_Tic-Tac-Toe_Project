import  cv2
import  numpy  as  np
import copy

ACTIVE = True
INACTIVE = False

ISMOVE = True
NOTMOVE = False



def move_test(img_block):
    
    gray_block  =  cv2.cvtColor(img_block,cv2.COLOR_BGR2GRAY)
    #dst_block = cv2.equalizeHist(gray_block)
    # ret_block, binary_block = cv2.threshold(gray_block, 100, 255, cv2.THRESH_BINARY_INV)
    binary_block=cv2.adaptiveThreshold(gray_block,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY_INV,5,10)
    cv2.imwrite('blockmovetest_pres.jpg',binary_block)
    # print(ret_block)
    # M_block, dev_block = cv2.meanStdDev(binary_block)
    # print(M_block)
    sum_block = binary_block.sum()
    print(sum_block)

    # cv2.imshow('test', binary_block)
    # cv2.waitKey(100000)
    # cv2.destroyAllWindows()

    

    if sum_block >= 6000:
        cv2.imwrite('blockmovetest.jpg',binary_block)
        return ISMOVE
    elif sum_block < 6000:
        return NOTMOVE



def player_move_read(img, list_area_fix, list_area_fix_dig, list_current_status):

    # img_T_m = cv2.imread('Tpl_m.jpg')
    # M2, dev2 = cv2.meanStdDev(img_T_m)
    # print(M2)

    # th_m = 1
    player_move = []
    list_current_status_new = copy.copy(list_current_status)

    flag_move_read = 0

    for i in range(len(list_current_status)):
        if list_current_status[i] == ACTIVE:
            # for ii in range(list_area_fix_dig[i][0][0], list_area_fix_dig[i][1][0]):
            #     for jj in range(list_area_fix_dig[i][0][1], list_area_fix_dig[i][1][1]):
            #         if (img[ii][jj][0] - M2[0])*(img[ii][jj][0] - M2[0])<=th and (img[ii][jj][1] - M2[1])*(img[ii][jj][1] - M2[1])<=th and (img[ii][jj][2] - M2[2])*(img[ii][jj][2] - M2[2])<=th:
            img_block = img[list_area_fix_dig[i][0][1]:list_area_fix_dig[i][1][1], list_area_fix_dig[i][0][0]:list_area_fix_dig[i][1][0]]
            is_move = move_test(img_block)
            if is_move == ISMOVE:
                player_move = i
                list_current_status_new[i] = INACTIVE
                flag_move_read = flag_move_read + 1
   
    if flag_move_read == 0:
        print('No move detected.')
    elif flag_move_read == 1:
        print('1 move detected.')
    else:
        print('Move detect error.')

    return player_move, list_current_status_new
