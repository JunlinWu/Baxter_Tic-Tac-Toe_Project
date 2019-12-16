import  cv2
import  numpy  as  np


def takeFirst(elem):
    return elem[0]



def takeSecond(elem):
    return elem[1]



def block_set(list_p, th_b):
    
    list_area = [[], [], [], [], [], [], [], [], []]

    list_p.sort(key = takeFirst)
    print(list_p)
    print('\n')
    
    list_boundary = [[], [], [], []]
    for i0 in range(len(list_p)):
        if i0/4 == 0:
            list_boundary[0].append(list_p[i0])
        if i0/4 == 1:
            list_boundary[1].append(list_p[i0])
        if i0/4 == 2:
            list_boundary[2].append(list_p[i0])
        if i0/4 == 3:
            list_boundary[3].append(list_p[i0])
    print(list_boundary)

    list_boundary[0].sort(key = takeSecond)
    list_boundary[1].sort(key = takeSecond)
    list_boundary[2].sort(key = takeSecond)
    list_boundary[3].sort(key = takeSecond)
    print(list_boundary)

    for k in range(len(list_area)):
        list_area[k] = [(list_boundary[k%3][k/3][0] + th_b, list_boundary[k%3][k/3][1] + th_b),
                         (list_boundary[k%3+1][k/3][0] - th_b, list_boundary[k%3+1][k/3][1] + th_b),
                         (list_boundary[k%3+1][k/3+1][0] - th_b, list_boundary[k%3+1][k/3+1][1] - th_b),
                         (list_boundary[k%3][k/3+1][0] + th_b, list_boundary[k%3][k/3+1][1] - th_b)]
    print(list_area)

    return list_area



def area_fix(list_area):
    list_area_fix = [[], [], [], [], [], [], [], [], []]
    list_area_fix_dig = [[], [], [], [], [], [], [], [], []]
    for i in range(len(list_area_fix)):
        list_area_fix[i] = [(list_area[i][0][0], list_area[i][1][1]), list_area[i][1], (list_area[i][1][0], list_area[i][2][1]), (list_area[i][0][0], list_area[i][2][1])]
        list_area_fix_dig[i] = [(list_area[i][0][0], list_area[i][1][1]), (list_area[i][1][0], list_area[i][2][1])]

    return list_area_fix, list_area_fix_dig
