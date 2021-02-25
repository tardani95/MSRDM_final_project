function [jacobi_abs_stack,jacobi_link_abs_stack,jacobi_cm_abs_stack] = DiffKinematics(fwd_kinematics,joint_types,cm_offset)
%DIFFKINEMATICS Summary of this function goes here
%   Detailed explanation goes here
    dof = length(joint_types);
    stack_size = size(fwd_kinematics,3) - 1; % substract -1 because of eye matrix
    cm_num = stack_size - dof;

    jacobi_abs_stack      = sym(zeros(6,dof,stack_size));
    jacobi_link_abs_stack = sym(zeros(6,dof,dof));
    jacobi_cm_abs_stack   = sym(zeros(6,dof,cm_num));
    
    % joint frames
    for stack_i = 1:dof
        for joint_i = 1:dof
            joint_type_i = joint_types(joint_i);
            if stack_i >= joint_i
                if joint_type_i == 'r'
                    jacobi_abs_stack(:,joint_i,stack_i) = Jacobi_revolute(fwd_kinematics(:,:,joint_i),fwd_kinematics(:,:,stack_i+1));
                else
                    jacobi_abs_stack(:,joint_i,stack_i) = Jacobi_prismatic(fwd_kinematics(:,:,joint_i),fwd_kinematics(:,:,stack_i+1));
                end
                jacobi_link_abs_stack(:,joint_i,stack_i) = simplify(jacobi_abs_stack(:,joint_i,stack_i));
            end
        end
    end
    
    % cm frames
    for stack_i = 1:cm_num
        for joint_i = 1:dof
            joint_type_i = joint_types(joint_i);
            if stack_i >= joint_i
                if joint_type_i == 'r'
                    jacobi_abs_stack(:,joint_i,dof+stack_i) = Jacobi_revolute(fwd_kinematics(:,:,joint_i),fwd_kinematics(:,:,dof+stack_i+1));
                else
                    jacobi_abs_stack(:,joint_i,dof+stack_i) = Jacobi_prismatic(fwd_kinematics(:,:,joint_i),fwd_kinematics(:,:,dof+stack_i+1));
                end
                jacobi_cm_abs_stack(:,joint_i,stack_i) = simplify(jacobi_abs_stack(:,joint_i,dof+stack_i));
            end
        end
    end
    
    
%     for stack_i = 1:stack_size
%         for joint_i = 1:dof
%             joint_type_i = joint_types(joint_i);
%             if joint_type_i == 'r'
%                 Jacobi_stack(:,joint_i,stack_i) = Jacobi_revolute(fwd_kinematics(:,:,joint_i),fwd_kinematics(:,:,stack_i))
%             else
%                 Jacobi_stack(:,joint_i,stack_i) = Jacobi_prismatic(fwd_kinematics(:,:,joint_i),fwd_kinematics(:,:,stack_i))
%             end
%         end
%     end
    

    function J_i = Jacobi_revolute(T_im1,wrtT)
        [~,z_0] = init_jacobi();
        
        z_im1 = tf2rotm(T_im1) * z_0;
        t_ref = tf2trvec(wrtT);
        t_im1 = tf2trvec(T_im1);
        t = t_ref - t_im1;
        
        Jvi = cross(z_im1,t);
        Jwi = z_im1;
        J_i = [Jvi;Jwi];
        
    end

    function J_i = Jacobi_prismatic(T_im1,~)
        [J_i,z_0] = init_jacobi();
        
        z_im1 = tf2rotm(T_im1) * z_0;
        J_i(1:3) = z_im1; 
    end

    function [J_i, z_0] = init_jacobi()
        J_i = sym(zeros(6,1));
        z_0 = sym([0,0,1]');
    end
end

