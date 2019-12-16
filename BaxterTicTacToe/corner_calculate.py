import  cv2
import  numpy  as  np
# import copy


def corner_calculate(img):
    ############ Read img ############
    # img  =  cv2.imread('1.jpg')
    weight = img.shape[0]
    height = img.shape[1]
    # channels = img.shape[2]
    print(height)
    print(weight)
    # print(img[0][0])
    

    ############ Extract board ############
    # img_T = cv2.imread('Tpl.jpg')
    # M1, dev1 = cv2.meanStdDev(img_T)
    # print(M1)
    # # print(img[13][56][0] - M1[0])
    # # print(img[13][56][1] - M1[1])
    # # print(img[13][56][2] - M1[2])

    #M1 = [[63.31574219], [72.13761719], [43.90777344]]
    #M1 = [[66.82400391], [73.51123047], [45.24900391]]
    M1 = [[73.7740625], [81.13773438], [50.84726562]]
    #M1 = [[69.60402734], [75.36183203], [47.48830859]]
    #M1 = [[69.60402734], [75.36183203], [50.84726562]]
    
    # th = 16  #1.jpg
    th = 170  # Control the diffence between R,G,B
    for ii in range(weight):
        for jj in range(height):
            # if abs(img[ii][jj][0] - M1[0])<=th and abs(img[ii][jj][1] - M1[1])<=th and abs(img[ii][jj][2] - M1[2])<=th:
            if (img[ii][jj][0] - M1[0])*(img[ii][jj][0] - M1[0])<=th and (img[ii][jj][1] - M1[1])*(img[ii][jj][1] - M1[1])<=th and (img[ii][jj][2] - M1[2])*(img[ii][jj][2] - M1[2])<=th:
                img[ii][jj][0] = 51
                img[ii][jj][1] = 255
                img[ii][jj][2] = 102
            else:
                img[ii][jj][0] = 0
                img[ii][jj][1] = 0
                img[ii][jj][2] = 0

    # kernel = np.ones((10,10),np.uint8)
    # img = cv2.erode(img,kernel,iterations = 1)
    cv2.imwrite('extractboard.jpg',img)


    ############ Hough Line ############
    list_k_line = []
    list_b_line = []

    gray  =  cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges  =  cv2.Canny(gray,50,150,apertureSize  =  3)

    # lines  =  cv2.HoughLines(gray,1,np.pi/180,200)
    lines  =  cv2.HoughLines(edges,1,np.pi/180,80)

    for i in range(0,len(lines)):
        rho,theta = lines[i][0][0],lines[i][0][1]
        a  =  np.cos(theta)
        b  =  np.sin(theta)
        x0  =  a*rho
        y0  =  b*rho
        x1  =  int(x0  +  1000*(-b))
        y1  =  int(y0  +  1000*(a))
        x2  =  int(x0  -  1000*(-b))
        y2  =  int(y0  -  1000*(a))
        if (x2 - x1) == 0:
            continue
        k_line = float(y2-y1)/float(x2-x1)
        b_line = float(y1) - k_line*float(x1)

        list_k_line.append(k_line)
        list_b_line.append(b_line)

        cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
    print(list_k_line)
    print(list_b_line)
    print('\n')
    cv2.imwrite('houghlines.jpg',img)


    ############ Line Cluster ############
    # 1.jpg
    # list_k_line = [4.326666666666667, 4.008264462809917, -0.1404040404040404, 5.674351585014409, -0.4039913700107875, 4.006198347107438]
    # list_b_line = [-2297.1400000000003, -2154.181818181818, 480.26666666666665, -2331.7291066282423, 439.31067961165047, -2106.568181818182]

    # list_k_line = [-0.10513078470824949, 28.542857142857144, -0.10513078470824949, -0.10513078470824949, 28.542857142857144, -0.08734939759036145, -0.12298387096774194]
    # list_b_line = [1019.6438631790745, -30064.085714285717, 1166.2208249496982, 1164.2208249496982, -33693.02857142857, 1139.6475903614457, 1052.6189516129032]
    # list_k_line_Beforesort = copy.copy(list_k_line)
    # list_k_line.sort()
    # list_k_index = []
    # for kk in range(len(list_k_line)):
    #     k_index = list_k_line_Beforesort.index(list_k_line[kk])
    #     list_k_index.append(k_index)
    # print(list_k_line_Beforesort)
    # print(list_k_line)
    # print(list_k_index)

    # list_line_cluster0 = []
    # list_line_cluster1 = []
    # list_line_cluster2 = []
    # list_line_cluster3 = []
    # list_line_cluster4 = []
    # list_line_cluster5 = []
    # list_line_cluster6 = []
    # list_line_cluster7 = []
    
    # list_line_cluster = [list_line_cluster0, list_line_cluster1, list_line_cluster2, list_line_cluster3, list_line_cluster4, list_line_cluster5, list_line_cluster6, list_line_cluster7]
    list_line_cluster = [[], [], [], [], [], [], [], []]
    list_line_cluster[0].append([list_k_line[0], list_b_line[0]])
    g_emp = 1
    
    th2 = 1  # Control the diffence between horizontal k
    th3 = 50  # Control the diffence between b -u&v
    th4 = 50  # Control the diffence between vertical k

    for kk in range(len(list_k_line)):
        flag = False
        for ww in range(len(list_line_cluster)):
            if list_line_cluster[ww] == []:
                break
            # if (list_k_line[kk]-list_line_cluster[ww][0][0])*(list_k_line[kk]-list_line_cluster[ww][0][0]) + (list_b_line[kk]-list_line_cluster[ww][0][1])*(list_b_line[kk]-list_line_cluster[ww][0][1]) < th2:
            if abs(list_k_line[kk])<=1:
                if abs(list_line_cluster[ww][0][0])>1:
                    continue
                if abs(abs(list_k_line[kk])-abs(list_line_cluster[ww][0][0]))<=th2 and abs(list_k_line[kk]*(weight/2)+list_b_line[kk]-list_line_cluster[ww][0][0]*(weight/2)-list_line_cluster[ww][0][1])<=th3:
                    list_line_cluster[ww].append([list_k_line[kk], list_b_line[kk]])
                    flag = True
                    break
            else:
                if abs(list_line_cluster[ww][0][0])<=1:
                    continue
                if abs(abs(list_k_line[kk])-abs(list_line_cluster[ww][0][0]))<=th4 and abs((((height/2)-list_b_line[kk])/list_k_line[kk])-(((height/2)-list_line_cluster[ww][0][1])/list_line_cluster[ww][0][0]))<=th3:
                    list_line_cluster[ww].append([list_k_line[kk], list_b_line[kk]])
                    flag = True
                    break
        if flag == False:
            list_line_cluster[g_emp].append([list_k_line[kk], list_b_line[kk]])
            g_emp = g_emp + 1

    print(list_line_cluster)
    print('\n')



    ############ Line Calc ############
    list_line = []
    for rr in range(len(list_line_cluster)):
        ex_line = np.mean(list_line_cluster[rr], axis=0)
        list_line.append(ex_line)

    print(list_line)
    # print(list_line[2][1])
    print('\n')


    ############ Point Calc ############
    list_point = []
    for tt in range(len(list_line)):
        for ss in range(tt+1, len(list_line)):
            if (list_line[ss][0] - list_line[tt][0]) == 0:
                continue
            xx = -((list_line[ss][1] - list_line[tt][1])/(list_line[ss][0] - list_line[tt][0]))
            yy = list_line[tt][0]*xx + list_line[tt][1]
            if xx<0 or yy<0 or yy>weight or xx>height:
                continue
            list_point.append((int(xx), int(yy)))

    print(list_point)
    print('\n')


    ############ Visualize Point ############
    point_size = 5
    point_color = (255, 0, 0) # BGR
    thickness = 4 #0/4/8

    for point in list_point:
        cv2.circle(img, point, point_size, point_color, thickness)

    cv2.imwrite('result.jpg',img)

    return list_point
