import numpy as np

z_offset = 0.1765
x_offset = 0.07
z_protection_offset=0.02
offset = np.array([0.0, 0.0, z_offset])
container_bottom_z = 0.08833251219030998-z_offset
container_top_z = 0.23337363864356647-z_offset

#pick_area_right_front=[0.6503643796355792-x_offset, 0.27546839699130893, 0.23337363864356647-z_offset]
#pick_area_left_back=[0.49038050737187766-x_offset, 0.0033654509754437343, 0.23354094197136313-z_offset]

#pack_area_right_front=[0.6585063036993619-x_offset, -0.04040245879285586, 0.23514989992182642-z_offset]
#pack_area_left_back=[0.29793577849259717-x_offset, -0.3063929293623967, 0.2330777527633756-z_offset]

def get_limit_area(mode):
    if mode == 'pick':
        corner_poses = np.array([pick_area_right_front, pick_area_right_back, \
                    pick_area_left_front, pick_area_left_back])
    elif mode == 'pack':
        corner_poses = np.array([pack_area_right_front, pack_area_right_back, \
                    pack_area_left_front, pack_area_left_back])
    limit_area =np.array([\
        np.min(corner_poses[:,0]), np.max(corner_poses[:,0]), \
        np.min(corner_poses[:,1]), np.max(corner_poses[:,1]), \
        container_bottom_z, container_top_z])
    return limit_area
    

pick_area_right_front = np.array([0.594847462059874, 0.14274090757894567, 0.09079485467748898])-offset
pick_area_right_back = np.array([0.38938066183683273, 0.1417364415408569, 0.0855291017561234])-offset
pick_area_left_back = np.array([0.3853168486735845, 2.390630334030992e-05, 0.08778242546199484])-offset
pick_area_left_front = np.array([0.5933161328562441, 0.004832332146942052, 0.09334491064317658])-offset
pick_limit = get_limit_area('pick')


pack_area_right_front = np.array([0.6007076681949359, -0.052942668953708145, 0.09258841682467059])-offset
pack_area_right_back = np.array([0.3796510003590328, -0.043681487746443065, 0.08833251219030998])-offset
pack_area_left_back = np.array([0.3862908559403928, -0.17777213624815325, 0.09019875491843092])-offset
pack_area_left_front = np.array([0.5853305702449692, -0.17273277686932115, 0.09361630700578974])-offset
pack_limit = get_limit_area('pack')


def get_center(mode):
    #x_min, x_max, y_min, y_max, z_min, z_max
    if mode == 'pick':
        return np.mean(pick_limit.reshape(3,2), axis=-1)
    elif mode == 'pack':
        return np.mean(pack_limit.reshape(3,2), axis=-1)

def get_relative_point_base_location(relative_i, relative_j, relative_k, mode):
    if mode == 'pick':
        area_limit = pick_limit
    elif mode == 'pack':
        area_limit = pack_limit
    if relative_i > (area_limit[1]-area_limit[0]) * 100.0 or \
        relative_j > (area_limit[3]-area_limit[2]) * 100.0 or \
        relative_i < 0 or relative_j < 0 or relative_k < 0:
        print ("ERROR! The relative location is out of the limit area!")
    else:
        return [area_limit[0]+relative_i/100.0, \
                area_limit[2]+relative_j/100.0, \
                area_limit[4]+relative_k/100.0]
