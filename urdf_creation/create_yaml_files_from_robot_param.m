clc;clear;

parameters;
import yaml.*;

param_robot.holder.R__joint = eye(3);
param_robot.holder.d__joint = [0 param_robot.l__I 0];
param_robot.holder.d__link = [0 0 0];
param_robot.holder.axis__joint = [0 0 1];
param_robot.holder.limits__joint = [-pi pi];
param_robot.holder.R__inertiaOrigin = eye(3);
param_robot.holder.d__inertiaOrigin = [0 0 0];
param_robot.holder.d__inertiaMatrixDiag = param_robot.inertia_matrix_sI;
param_robot.holder.m = param_robot.mI;

param_robot.link1.R__joint = eye(3);
param_robot.link1.d__joint = [param_robot.l__1 0 0];
param_robot.link1.d__link = [param_robot.l__s1 0 0];
param_robot.link1.axis__joint = [0 0 1];
param_robot.link1.limits__joint = [-pi pi];
param_robot.link1.R__inertiaOrigin = eye(3);
param_robot.link1.d__inertiaOrigin = [0 0 0];
param_robot.link1.d__inertiaMatrixDiag = param_robot.inertia_matrix_s1;
param_robot.link1.m = param_robot.m1;

param_robot.link2.R__joint = eye(3);
param_robot.link2.d__joint = [param_robot.l__2 0 0];
param_robot.link2.d__link = [param_robot.l__s2 0 0];
param_robot.link2.axis__joint = [0 0 1];
param_robot.link2.limits__joint = [0 0];
param_robot.link2.R__inertiaOrigin = eye(3);
param_robot.link2.d__inertiaOrigin = [0 0 0];
param_robot.link2.d__inertiaMatrixDiag = param_robot.inertia_matrix_s2;
param_robot.link2.m = param_robot.m2;

yaml.dumpFile("./urdf_creation/param_robot.yaml", param_robot)
%yaml.dump(param_robot)

% xacro [OPTIONS] <input_file> [OUTPUT_FILE]
%pyrun('xacro 2dof_sys.xacro > 2dof_sys.urdf');
%python xacro.py -o ./2dof_sys.urdf 2dof_sys.xacro