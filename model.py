import Adams
import os
import numpy as np

current_path = os.getcwd()
mnf_path = current_path + '/model/file.mnf'
beam_number = 2 # 柔性梁的个数 
beam_length = 29.64 # 柔性梁的长度, 轴向沿Z轴


# ADAMS 基本配置
Model = Adams.getCurrentModel()
Units = Adams.defaults.units.setUnits(length = 'meter', mass = 'kg', time = 'second',  angle = 'radians', force = 'newton', frequency = 'hz')
material_satellite = Model.Materials.create(name = 'Sat_material', density = 120, youngs_modulus = 1.1E+11, poissons_ratio = 0.33)
material_nonMass = Model.Materials.create(name = 'nonMass', density = 1E-4, youngs_modulus = 1.1E+11, poissons_ratio = 0.33)
# 在设置时默认无重力
color_list = ['RED', 'GREEN', 'YELLOW', 'PINK', 'ORANGE', 'PINK']

ground = Model.Parts['ground']
marker_origin = ground.Markers.create(name = 'origin', position = [0, 0, 0], orientation = [0, 0, 0]) 

# 柔性梁导入
for i in range(1, beam_number+1):
    beam_name = 'beam' + str(i)
    beam_pos = [0, 0, beam_length*(i-1)]
    beam_orien = [0, 0, 0]

    beam = Model.Parts.createFlexBody(name = beam_name, modal_neutral_file_name = mnf_path, location = beam_pos)

# 建立无质量的刚体并添加球铰
for i in range(1, beam_number): 
    part_name = 'nonMass_' + str(i)
    beam_i = Adams.stoo('beam' + str(i))
    beam_j = Adams.stoo('beam' + str(i+1))
    marker_i = beam_i.Markers['INT_NODE_27181']
    marker_j = beam_j.Markers['INT_NODE_27180']
    spericalLocation = np.array(beam_i.location) + np.array(marker_i.location)
    
    nonMass = Model.Parts.createRigidBody(name = part_name)
    Marker_center = nonMass.Markers.create(name = 'center', location = spericalLocation.tolist(), orientation = [0, 0, 0])
    nonMass.Geometries.createEllipsoid(name = 'ellipsoid', center_marker = Marker_center, x_scale_factor = 0.5, y_scale_factor = 0.5, z_scale_factor = 0.5)

    spherical_i_nonMass = Model.Constraints.createSpherical(name = 'spherical_' + str(i) +  '_non', i_part = beam_i, j_part = nonMass, location = Marker_center.location, orientation = [0, 0, 0])

    spherical_j_nonMass = Model.Constraints.createSpherical(name = 'spherical_non_' + str(i+1), i_part = nonMass, j_part = beam_j, location = Marker_center.location, orientation = [0, 0, 0])

# # 球铰建立
# for i in range(1, beam_number):
#     beam_i = Adams.stoo('beam' + str(i))
#     beam_j = Adams.stoo('beam' + str(i+1))

#     marker_i = beam_i.Markers['INT_NODE_27181']
#     marker_j = beam_j.Markers['INT_NODE_27180']

#     spherical_i_j = Model.Constraints.createSpherical(name = 'spherical' + str(i) + '_' + str(i+1), i_part = beam_i, j_part = beam_j, location = marker_i.location, orientation = [0,0,0])


# 建立卫星
for i in range(1, beam_number+1):
    sat_attach = Adams.stoo('beam' + str(i))

    name_satellite_1 = 'sat' + '_' + str(i) + '_1'
    name_satellite_2 = 'sat' + '_' + str(i) + '_2' 
    name_satellite_3 = 'sat' + '_' + str(i) + '_3'

    marker_ref_sat1 = sat_attach.Markers['INT_NODE_27183']
    marker_ref_sat2 = sat_attach.Markers['INT_NODE_27182']
    marker_ref_sat3 = sat_attach.Markers['INT_NODE_27184']

    satellite_1 = Model.Parts.createRigidBody(name = name_satellite_1)
    satellite_2 = Model.Parts.createRigidBody(name = name_satellite_2)
    satellite_3 = Model.Parts.createRigidBody(name = name_satellite_3)
    
    corner_1_location = np.array(sat_attach.location) + np.array(marker_ref_sat1.location) - np.array([0.5, 0.5, 0.5])
    corner_2_location = np.array(sat_attach.location) + np.array(marker_ref_sat2.location) - np.array([0.5, 0.5, 0.5])
    corner_3_location = np.array(sat_attach.location) + np.array(marker_ref_sat3.location) - np.array([0.5, 0.5, 0.5])
    print(marker_ref_sat1.location)
    # 中间节点为卫星实体，两侧节点为无质量单元
    # satellite1
    corner_marker_sat1 = satellite_1.Markers.create(name = 'corner_marker', location = corner_1_location.tolist(), orientation = [0,0,0])
    satellite_1.material_type = material_nonMass
    satellite_1.Geometries.createBlock(name = name_satellite_1 + 'body',  corner_marker = corner_marker_sat1, x = 1, y =1, z = 1)
    # satellite2
    corner_marker_sat2 = satellite_2.Markers.create(name = 'corner_marker', location = corner_2_location.tolist(), orientation = [0,0,0])
    satellite_2.material_type = material_satellite
    satellite_2.Geometries.createBlock(name = name_satellite_2 + 'body',  corner_marker = corner_marker_sat2, x = 1, y = 1, z = 1)
    # satellite3
    corner_marker_sat3 = satellite_3.Markers.create(name = 'corner_marker', location = corner_3_location.tolist(), orientation = [0,0,0])
    satellite_3.material_type = material_nonMass
    satellite_3.Geometries.createBlock(name = name_satellite_3 + 'body',  corner_marker = corner_marker_sat3, x = 1, y = 1, z = 1)

    # 添加卫星和桁架的绑定
    name_Joint_1 = 'FixedJoint_' + str(i) + '_1'
    name_Joint_2 = 'FixedJoint_' + str(i) + '_2'
    name_Joint_3 = 'FixedJoint_' + str(i) + '_3'
    
    FixedJoint_1 = Model.Constraints.createFixed(name = name_Joint_1, i_part = sat_attach, j_part = satellite_1, location = satellite_1.Markers['cm'].location, orientation = [0,0,0])
    FixedJoint_2 = Model.Constraints.createFixed(name = name_Joint_2, i_part = sat_attach, j_part = satellite_2, location = satellite_2.Markers['cm'].location, orientation = [0,0,0])
    FixedJoint_3 = Model.Constraints.createFixed(name = name_Joint_3, i_part = sat_attach, j_part = satellite_3, location = satellite_3.Markers['cm'].location, orientation = [0,0,0])

    # 建立卫星的输入变量
    Fx_1 = Model.DataElements.createStateVariable(name = name_satellite_1 + '_Fx', initial_condition = 0, function = '0')
    Fy_1 = Model.DataElements.createStateVariable(name = name_satellite_1 + '_Fy', initial_condition = 0, function = '0')
    Fz_1 = Model.DataElements.createStateVariable(name = name_satellite_1 + '_Fz', initial_condition = 0, function = '0')
    Tx_1 = Model.DataElements.createStateVariable(name = name_satellite_1 + '_Tx', initial_condition = 0, function = '0')
    Ty_1 = Model.DataElements.createStateVariable(name = name_satellite_1 + '_Ty', initial_condition = 0, function = '0')
    Tz_1 = Model.DataElements.createStateVariable(name = name_satellite_1 + '_Tz', initial_condition = 0, function = '0')

    Fx_2 = Model.DataElements.createStateVariable(name = name_satellite_2 + '_Fx', initial_condition = 0, function = '0')
    Fy_2 = Model.DataElements.createStateVariable(name = name_satellite_2 + '_Fy', initial_condition = 0, function = '0')
    Fz_2 = Model.DataElements.createStateVariable(name = name_satellite_2 + '_Fz', initial_condition = 0, function = '0')
    Tx_2 = Model.DataElements.createStateVariable(name = name_satellite_2 + '_Tx', initial_condition = 0, function = '0')
    Ty_2 = Model.DataElements.createStateVariable(name = name_satellite_2 + '_Ty', initial_condition = 0, function = '0')
    Tz_2 = Model.DataElements.createStateVariable(name = name_satellite_2 + '_Tz', initial_condition = 0, function = '0')

    Fx_3 = Model.DataElements.createStateVariable(name = name_satellite_3 + '_Fx', initial_condition = 0, function = '0')
    Fy_3 = Model.DataElements.createStateVariable(name = name_satellite_3 + '_Fy', initial_condition = 0, function = '0')
    Fz_3 = Model.DataElements.createStateVariable(name = name_satellite_3 + '_Fz', initial_condition = 0, function = '0')
    Tx_3 = Model.DataElements.createStateVariable(name = name_satellite_3 + '_Tx', initial_condition = 0, function = '0')
    Ty_3 = Model.DataElements.createStateVariable(name = name_satellite_3 + '_Ty', initial_condition = 0, function = '0')
    Tz_3 = Model.DataElements.createStateVariable(name = name_satellite_3 + '_Tz', initial_condition = 0, function = '0')

    # 创建集中力
    force_name_1 = 'F_' + str(i) + '_1'
    force_name_2 = 'F_' + str(i) + '_2'
    force_name_3 = 'F_' + str(i) + '_3'

    float_marker_name_1 = 'MARKER_Float_' + str(i) + '_1'
    float_marker_name_2 = 'MARKER_Float_' + str(i) + '_2'
    float_marker_name_3 = 'MARKER_Float_' + str(i) + '_3'
    float_marker_1 = ground.FloatingMarkers.create(name = float_marker_name_1)
    float_marker_2 = ground.FloatingMarkers.create(name = float_marker_name_2)
    float_marker_3 = ground.FloatingMarkers.create(name = float_marker_name_3)
    
    marker_ref_1 = ground.Markers.create(name='MARKER_ref_'+str(i)+'_1', location = satellite_1.Markers["cm"].location, orientation = [0,0,0])
    marker_ref_2 = ground.Markers.create(name='MARKER_ref_'+str(i)+'_2', location = satellite_2.Markers["cm"].location, orientation = [0,0,0])
    marker_ref_3 = ground.Markers.create(name='MARKER_ref_'+str(i)+'_3', location = satellite_3.Markers["cm"].location, orientation = [0,0,0])
    
    generalForce_1 = Model.Forces.createGeneralForce(name = force_name_1, i_marker = satellite_1.Markers["cm"], j_floating_marker = float_marker_1, ref_marker = marker_ref_1)
    generalForce_1.x_force_function  = 'VARVAL(' +  name_satellite_1 + '_Fx' + ')'
    generalForce_1.y_force_function  = 'VARVAL(' +  name_satellite_1 + '_Fy' + ')'
    generalForce_1.z_force_function  = 'VARVAL(' +  name_satellite_1 + '_Fz' + ')'
    generalForce_1.x_torque_function  = 'VARVAL(' +  name_satellite_1 + '_Tx' + ')'
    generalForce_1.y_torque_function  = 'VARVAL(' +  name_satellite_1 + '_Ty' + ')'
    generalForce_1.z_torque_function  = 'VARVAL(' +  name_satellite_1 + '_Tz' + ')'

    generalForce_2 = Model.Forces.createGeneralForce(name = force_name_2, i_marker = satellite_2.Markers["cm"], j_floating_marker = float_marker_2, ref_marker = marker_ref_2)
    generalForce_2.x_force_function  = 'VARVAL(' +  name_satellite_2 + '_Fx' + ')'
    generalForce_2.y_force_function  = 'VARVAL(' +  name_satellite_2 + '_Fy' + ')'
    generalForce_2.z_force_function  = 'VARVAL(' +  name_satellite_2 + '_Fz' + ')'
    generalForce_2.x_torque_function  = 'VARVAL(' +  name_satellite_2 + '_Tx' + ')'
    generalForce_2.y_torque_function  = 'VARVAL(' +  name_satellite_2 + '_Ty' + ')'
    generalForce_2.z_torque_function  = 'VARVAL(' +  name_satellite_2 + '_Tz' + ')'

    generalForce_3 = Model.Forces.createGeneralForce(name = force_name_3, i_marker = satellite_3.Markers["cm"], j_floating_marker = float_marker_3, ref_marker = marker_ref_3)
    generalForce_3.x_force_function  = 'VARVAL(' +  name_satellite_3 + '_Fx' + ')'
    generalForce_3.y_force_function  = 'VARVAL(' +  name_satellite_3 + '_Fy' + ')'
    generalForce_3.z_force_function  = 'VARVAL(' +  name_satellite_3 + '_Fz' + ')'
    generalForce_3.x_torque_function  = 'VARVAL(' +  name_satellite_3 + '_Tx' + ')'
    generalForce_3.y_torque_function  = 'VARVAL(' +  name_satellite_3 + '_Ty' + ')'
    generalForce_3.z_torque_function  = 'VARVAL(' +  name_satellite_3 + '_Tz' + ')'

    # 建立测量
    pos_x_1_name = 'beam_' + str(i) + '_1_pos_x'
    pos_y_1_name = 'beam_' + str(i) + '_1_pos_y'
    pos_z_1_name = 'beam_' + str(i) + '_1_pos_z'
    pos_x_2_name = 'beam_' + str(i) + '_2_pos_x'
    pos_y_2_name = 'beam_' + str(i) + '_2_pos_y'
    pos_z_2_name = 'beam_' + str(i) + '_2_pos_z'
    pos_x_3_name = 'beam_' + str(i) + '_3_pos_x'
    pos_y_3_name = 'beam_' + str(i) + '_3_pos_y'
    pos_z_3_name = 'beam_' + str(i) + '_3_pos_z'
    vel_x_1_name = 'beam_' + str(i) + '_1_vel_x'
    vel_y_1_name = 'beam_' + str(i) + '_1_vel_y'
    vel_z_1_name = 'beam_' + str(i) + '_1_vel_z'
    vel_x_2_name = 'beam_' + str(i) + '_2_vel_x'
    vel_y_2_name = 'beam_' + str(i) + '_2_vel_y'
    vel_z_2_name = 'beam_' + str(i) + '_2_vel_z'
    vel_x_3_name = 'beam_' + str(i) + '_3_vel_x'
    vel_y_3_name = 'beam_' + str(i) + '_3_vel_y'
    vel_z_3_name = 'beam_' + str(i) + '_3_vel_z'
    # 相对位置测量
    pos_x_1 = Model.Measures.createPoint(name = pos_x_1_name, point = marker_ref_sat1, characteristic = 'translational_displacement', component = 'x_component')
    pos_y_1 = Model.Measures.createPoint(name = pos_y_1_name, point = marker_ref_sat1, characteristic = 'translational_displacement', component = 'y_component')
    pos_z_1 = Model.Measures.createPoint(name = pos_z_1_name, point = marker_ref_sat1, characteristic = 'translational_displacement', component = 'z_component')
    pos_x_2 = Model.Measures.createPoint(name = pos_x_2_name, point = marker_ref_sat2, characteristic = 'translational_displacement', component = 'x_component')
    pos_y_2 = Model.Measures.createPoint(name = pos_y_2_name, point = marker_ref_sat2, characteristic = 'translational_displacement', component = 'y_component')
    pos_z_2 = Model.Measures.createPoint(name = pos_z_2_name, point = marker_ref_sat2, characteristic = 'translational_displacement', component = 'z_component')
    pos_x_3 = Model.Measures.createPoint(name = pos_x_3_name, point = marker_ref_sat3, characteristic = 'translational_displacement', component = 'x_component')
    pos_y_3 = Model.Measures.createPoint(name = pos_y_3_name, point = marker_ref_sat3, characteristic = 'translational_displacement', component = 'y_component')
    pos_z_3 = Model.Measures.createPoint(name = pos_z_3_name, point = marker_ref_sat3, characteristic = 'translational_displacement', component = 'z_component')

    
    vel_x_1 = Model.Measures.createPoint(name = vel_x_1_name, point = marker_ref_sat1, characteristic = 'translational_velocity', component = 'x_component')
    vel_y_1 = Model.Measures.createPoint(name = vel_y_1_name, point = marker_ref_sat1, characteristic = 'translational_velocity', component = 'y_component')
    vel_z_1 = Model.Measures.createPoint(name = vel_z_1_name, point = marker_ref_sat1, characteristic = 'translational_velocity', component = 'z_component')
    vel_x_2 = Model.Measures.createPoint(name = vel_x_2_name, point = marker_ref_sat2, characteristic = 'translational_velocity', component = 'x_component')
    vel_y_2 = Model.Measures.createPoint(name = vel_y_2_name, point = marker_ref_sat2, characteristic = 'translational_velocity', component = 'y_component')
    vel_z_2 = Model.Measures.createPoint(name = vel_z_2_name, point = marker_ref_sat2, characteristic = 'translational_velocity', component = 'z_component')
    vel_x_3 = Model.Measures.createPoint(name = vel_x_3_name, point = marker_ref_sat3, characteristic = 'translational_velocity', component = 'x_component')
    vel_y_3 = Model.Measures.createPoint(name = vel_y_3_name, point = marker_ref_sat3, characteristic = 'translational_velocity', component = 'y_component')
    vel_z_3 = Model.Measures.createPoint(name = vel_z_3_name, point = marker_ref_sat3, characteristic = 'translational_velocity', component = 'z_component')

    Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + pos_x_1_name)
    Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + pos_y_1_name)
    Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + pos_z_1_name)
    Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + vel_x_1_name)
    Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + vel_y_1_name)
    Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + vel_z_1_name)
    Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + pos_x_2_name)
    Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + pos_y_2_name)
    Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + pos_z_2_name)
    Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + vel_x_2_name)
    Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + vel_y_2_name)
    Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + vel_z_2_name)
    Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + pos_x_3_name)
    Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + pos_y_3_name)
    Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + pos_z_3_name)
    Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + vel_x_3_name)
    Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + vel_y_3_name)
    Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + vel_z_3_name)

    # 测量相关的变量
    out_pos_x_1_name = 'pos_x_' + str(i) + '_1'
    out_pos_y_1_name = 'pos_y_' + str(i) + '_1'
    out_pos_z_1_name = 'pos_z_' + str(i) + '_1'
    out_vel_x_1_name = 'vel_x_' + str(i) + '_1'
    out_vel_y_1_name = 'vel_y_' + str(i) + '_1'
    out_vel_z_1_name = 'vel_z_' + str(i) + '_1'
    out_pos_x_2_name = 'pos_x_' + str(i) + '_2'
    out_pos_y_2_name = 'pos_y_' + str(i) + '_2'
    out_pos_z_2_name = 'pos_z_' + str(i) + '_2'
    out_vel_x_2_name = 'vel_x_' + str(i) + '_2'
    out_vel_y_2_name = 'vel_y_' + str(i) + '_2'
    out_vel_z_2_name = 'vel_z_' + str(i) + '_2'
    out_pos_x_3_name = 'pos_x_' + str(i) + '_3'
    out_pos_y_3_name = 'pos_y_' + str(i) + '_3'
    out_pos_z_3_name = 'pos_z_' + str(i) + '_3'
    out_vel_x_3_name = 'vel_x_' + str(i) + '_3'
    out_vel_y_3_name = 'vel_y_' + str(i) + '_3'
    out_vel_z_3_name = 'vel_z_' + str(i) + '_3'

    out_pos_x_1 = Model.DataElements.createStateVariable(name = out_pos_x_1_name, initial_condition = 0, function = pos_x_1_name)
    out_pos_y_1 = Model.DataElements.createStateVariable(name = out_pos_y_1_name, initial_condition = 0, function = pos_y_1_name)
    out_pos_z_1 = Model.DataElements.createStateVariable(name = out_pos_z_1_name, initial_condition = 0, function = pos_z_1_name)
    out_vel_x_1 = Model.DataElements.createStateVariable(name = out_vel_x_1_name, initial_condition = 0, function = vel_x_1_name)
    out_vel_y_1 = Model.DataElements.createStateVariable(name = out_vel_y_1_name, initial_condition = 0, function = vel_y_1_name)
    out_vel_z_1 = Model.DataElements.createStateVariable(name = out_vel_z_1_name, initial_condition = 0, function = vel_z_1_name)
    out_pos_x_2 = Model.DataElements.createStateVariable(name = out_pos_x_2_name, initial_condition = 0, function = pos_x_2_name)
    out_pos_y_2 = Model.DataElements.createStateVariable(name = out_pos_y_2_name, initial_condition = 0, function = pos_y_2_name)
    out_pos_z_2 = Model.DataElements.createStateVariable(name = out_pos_z_2_name, initial_condition = 0, function = pos_z_2_name)
    out_vel_x_2 = Model.DataElements.createStateVariable(name = out_vel_x_2_name, initial_condition = 0, function = vel_x_2_name)
    out_vel_y_2 = Model.DataElements.createStateVariable(name = out_vel_y_2_name, initial_condition = 0, function = vel_y_2_name)
    out_vel_z_2 = Model.DataElements.createStateVariable(name = out_vel_z_2_name, initial_condition = 0, function = vel_z_2_name)
    out_pos_x_3 = Model.DataElements.createStateVariable(name = out_pos_x_3_name, initial_condition = 0, function = pos_x_3_name)
    out_pos_y_3 = Model.DataElements.createStateVariable(name = out_pos_y_3_name, initial_condition = 0, function = pos_y_3_name)
    out_pos_z_3 = Model.DataElements.createStateVariable(name = out_pos_z_3_name, initial_condition = 0, function = pos_z_3_name)
    out_vel_x_3 = Model.DataElements.createStateVariable(name = out_vel_x_3_name, initial_condition = 0, function = vel_x_3_name)
    out_vel_y_3 = Model.DataElements.createStateVariable(name = out_vel_y_3_name, initial_condition = 0, function = vel_y_3_name)
    out_vel_z_3 = Model.DataElements.createStateVariable(name = out_vel_z_3_name, initial_condition = 0, function = vel_z_3_name)

    
    # 角度相关测量 直接测量卫星的姿态 输出为四元数
    
    angle_ref = satellite_2.Markers.create(name = 'angle_ref_' + str(i), location = satellite_2.Markers['cm'].location, orientation = [0,0,0] )
    orient_w_name = 'sat_' + str(i) + '_2_orient_w'
    orient_x_name = 'sat_' + str(i) + '_2_orient_x'
    orient_y_name = 'sat_' + str(i) + '_2_orient_y'
    orient_z_name = 'sat_' + str(i) + '_2_orient_z'
    angleVel_x_name = 'sat_' + str(i) + '_2_angleVel_x'
    angleVel_y_name = 'sat_' + str(i) + '_2_angleVel_y'
    angleVel_z_name = 'sat_' + str(i) + '_2_angleVel_z'

    # 基于类
    # orient_w = Model.Measures.createOrient(name = orient_w_name, characteristic = 'euler_parameters', component='param_1_component', to_frame = marker_ref_sat2, from_frame = marker_origin)
    # orient_x = Model.Measures.createOrient(name = orient_x_name, characteristic = 'euler_parameters', component='param_2_component', to_frame = marker_ref_sat2, from_frame = marker_origin)
    # orient_y = Model.Measures.createOrient(name = orient_y_name, characteristic = 'euler_parameters', component='param_3_component', to_frame = marker_ref_sat2, from_frame = marker_origin)
    # orient_z = Model.Measures.createOrient(name = orient_z_name, characteristic = 'euler_parameters', component='param_4_component', to_frame = marker_ref_sat2, from_frame = marker_origin)

    # angleVel_x = Model.Measures.createObject(name = angleVel_x_name, characteristic = 'angular_velocity', component = 'x_component', object = marker_ref_sat2)
    # angleVel_y = Model.Measures.createObject(name = angleVel_y_name, characteristic = 'angular_velocity', component = 'y_component', object = marker_ref_sat2)
    # angleVel_z = Model.Measures.createObject(name = angleVel_z_name, characteristic = 'angular_velocity', component = 'z_component', object = marker_ref_sat2)

    # Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + orient_w_name)
    # Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + orient_x_name)
    # Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + orient_y_name)
    # Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + orient_z_name)
    # Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + angleVel_x_name)
    # Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + angleVel_y_name)
    # Adams.execute_cmd("measure_display delete mea_display = ." + Model.name + "." + angleVel_z_name)

    # 基于命令行
    cmd = 'measure create point measure_name=.' + Model.name + '.' + angleVel_x_name + ' point= ' + angle_ref.name + ' characteristic = "angular_velocity"  component =  "x_component"      comments=""  create_measure_display = no'
    Adams.execute_cmd(cmd)
    cmd = 'measure create point measure_name=.' + Model.name + '.' + angleVel_y_name + ' point= ' + angle_ref.name + ' characteristic = "angular_velocity"  component =  "y_component"      comments=""  create_measure_display = no'
    Adams.execute_cmd(cmd)
    cmd = 'measure create point measure_name=.' + Model.name + '.' + angleVel_z_name + ' point= ' + angle_ref.name + ' characteristic = "angular_velocity"  component =  "z_component"      comments=""  create_measure_display = no'
    Adams.execute_cmd(cmd)

    cmd = 'measure create orient  measure_name =.' + Model.name + '.' + orient_w_name + ' component =  "param_1_component"  characteristic = euler_parameters  to_frame =' +  angle_ref.name + ' comments="" create_measure_display = no '
    Adams.execute_cmd(cmd)
    cmd = 'measure create orient  measure_name =.' + Model.name + '.' + orient_x_name + ' component =  "param_2_component"  characteristic = euler_parameters  to_frame =' +  angle_ref.name + ' comments="" create_measure_display = no '
    Adams.execute_cmd(cmd)
    cmd = 'measure create orient  measure_name =.' + Model.name + '.' + orient_y_name + ' component =  "param_3_component"  characteristic = euler_parameters  to_frame =' +  angle_ref.name + ' comments="" create_measure_display = no '
    Adams.execute_cmd(cmd)
    cmd = 'measure create orient  measure_name =.' + Model.name + '.' + orient_z_name + ' component =  "param_4_component"  characteristic = euler_parameters  to_frame =' +  angle_ref.name + ' comments="" create_measure_display = no '
    Adams.execute_cmd(cmd)

    out_orient_w_name = 'orient_w_2_' + str(i) 
    out_orient_x_name = 'orient_x_2_' + str(i) 
    out_orient_y_name = 'orient_y_2_' + str(i)
    out_orient_z_name = 'orient_z_2_' + str(i) 
    out_angleVel_x_name = 'angleVel_x_2_' + str(i)
    out_angleVel_y_name = 'angleVel_y_2_' + str(i)
    out_angleVel_z_name = 'angleVel_z_2_' + str(i)

    out_orient_w = Model.DataElements.createStateVariable(name = out_orient_w_name, initial_condition = 0, function = orient_w_name)
    out_orient_x = Model.DataElements.createStateVariable(name = out_orient_x_name, initial_condition = 0, function = orient_x_name)
    out_orient_y = Model.DataElements.createStateVariable(name = out_orient_y_name, initial_condition = 0, function = orient_y_name)
    out_orient_z = Model.DataElements.createStateVariable(name = out_orient_z_name, initial_condition = 0, function = orient_z_name)
    out_angleVel_x = Model.DataElements.createStateVariable(name = out_angleVel_x_name, initial_condition = 0, function = angleVel_x_name)
    out_angleVel_y = Model.DataElements.createStateVariable(name = out_angleVel_y_name, initial_condition = 0, function = angleVel_y_name)
    out_angleVel_z = Model.DataElements.createStateVariable(name = out_angleVel_z_name, initial_condition = 0, function = angleVel_z_name)
    
    
    PInput = Model.DataElements.createPInput(name = 'INPUT_beam_' + str(i), variable = [Fx_1, Fy_1, Fz_1, Tx_1, Ty_1, Tz_1, Fx_2, Fy_2, Fz_2, Tx_2, Ty_2, Tz_2, Fx_3, Fy_3, Fz_3, Tx_3, Ty_3, Tz_3])
    POutput = Model.DataElements.createPOutput(name = 'OUTPUT_beam_' + str(i), variable = [out_pos_x_1, out_pos_y_1, out_pos_z_1, out_vel_x_1, out_vel_y_1,out_vel_z_1, out_pos_x_2, out_pos_y_2, out_pos_z_2, out_vel_x_2, out_vel_y_2, out_vel_z_2, out_pos_x_3, out_pos_y_3, out_pos_z_3, out_vel_x_3, out_vel_y_3, out_vel_z_3, out_orient_w, out_orient_x, out_orient_y, out_orient_z, out_angleVel_x, out_angleVel_y, out_angleVel_z])














    


