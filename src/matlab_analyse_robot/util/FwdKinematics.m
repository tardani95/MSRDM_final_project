function [H_abs_stack,X_abs_stack,H_link_abs_stack,H_cm_abs_stack, H_link_rel_stack, H_cm_rel_stack, X_link_abs_stack, X_cm_abs_stack] = FwdKinematics(DH_link,DH_cm,cm_offset)
%FWDKINEMATICS Summary of this function goes here
%   Detailed explanation goes here
    dof = size(DH_link,1);
    cm_length = size(DH_cm,1);
    
    tot_length = dof+1 + cm_length;
    
    H_abs_stack = sym(zeros(4,4, tot_length));
    H_abs_stack(:,:,1) = sym(eye(4));
    
    H_link_rel_stack = sym(zeros(4,4, dof));
    H_link_abs_stack = sym(zeros(4,4, dof));
    
    H_cm_rel_stack = sym(zeros(4,4, cm_length));
    H_cm_abs_stack = sym(zeros(4,4, cm_length));
    
    X_abs_stack = sym(zeros(6,1, tot_length));
    X_link_abs_stack = sym(zeros(6,1, dof));
    X_cm_abs_stack = sym(zeros(6,1, cm_length));
    
    for i = 2 : dof+1
        H_link_rel_stack(:,:,i-1) = dh2rel_tf(DH_link(i-1,:));
        H_link_abs_stack(:,:,i-1) = H_abs_stack(:,:,i-1)*H_link_rel_stack(:,:,i-1);
        X_link_abs_stack(:,:,i-1) = tf2pose(H_link_abs_stack(:,:,i-1));
        
        H_abs_stack(:,:,i) = H_link_abs_stack(:,:,i-1);
        X_abs_stack(:,:,i) = X_link_abs_stack(:,:,i-1);
        
    end
    
    for i = 1 : cm_length
        H_cm_rel_stack(:,:,i) = dh2rel_tf(DH_cm(i,:));
        H_cm_abs_stack(:,:,i) = H_abs_stack(:,:,i+cm_offset)* H_cm_rel_stack(:,:,i);
        X_cm_abs_stack(:,:,i) = tf2pose(H_cm_abs_stack(:,:,i));
        
        H_abs_stack(:,:,i + dof+1) = H_cm_abs_stack(:,:,i);
        X_abs_stack(:,:,i + dof+1) = X_cm_abs_stack(:,:,i);
    end
end

