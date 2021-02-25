function [Y, Theta] = RobotRegressor(eqn,state_variables)
%ROBOTREGRESSOR Summary of this function goes here
%   Detailed explanation goes here
    dof = size(eqn,1);
    eqn = expand(eqn);
    parameters = [];
    
    for idx = 1:dof
        
        expr = children(eqn(idx));
        
        for jdx = 1:size(expr,2)
            
            parameters = [parameters remove_state_var(expr(jdx),state_variables)];
            
        end
    end
    
    Theta = unique(parameters);
    p = size(Theta,2);
    p_vec = sym('p%d',[1 p], 'positive');
    
    eqn = subs(eqn, Theta, p_vec);
    
    %form regressor matrix Y
    Y = simplify(equationsToMatrix(eqn, p_vec));
    Theta = Theta';
     
end
    
function [cut_exp, isempty] = remove_state_var(exp,state_vars)
    isempty = false;
%     exp = expand(exp);
    [subexps,main_issum,main_isprod,main_ispow,main_isfunc] = children2(exp);
    
    if main_ispow
        [~, isempty] = remove_state_var(subexps(1),state_vars);
        if isempty
            cut_exp = sym([]);
        else
            cut_exp = exp;
        end
        return;
    end
    
    cut_exp = sym([]);
    
    for sub_idx = 1:size(subexps,2)
        subexp = subexps(sub_idx);
        [childs,issum,isprod,ispow,isfunc] = children2(subexp);
        if size(childs,2) > 1 || ispow || isfunc
            [subexp, isempty] = remove_state_var(subexp,state_vars);
        else
           childs(isSymType(childs,'constant')) = sym([]);
           for state_var_idx = 1:size(state_vars,2)
               childs(childs == state_vars(state_var_idx)) = sym([]);
           end
                      
           subexp = childs;
        end
        
        cut_exp = [cut_exp subexp];
    end

    if main_issum
        cut_exp = sum(cut_exp);
    elseif main_isprod
        cut_exp = prod(cut_exp);
    elseif size(cut_exp,2) == 0
        isempty = true;
    else
        cut_exp = exp;
    end
%     cut_exp
end


function [childs, issum, isprod, ispow, isfun] = children2(parent)
%CHILDREN2 Returns the children plus meta info. of math. concatenation
% Author: tkorthals@cit-ec.uni-bielefeld.de
% Input
%  parent                   Symbolic expression
% Output
%  childs                   Vector of symbolic expression
%  issum                    True if parent is sum of children
%  isprod                   True if parent is product of children
%  ispow                    True if parent is power of children
%  isfun                    True if parent is some function's argument

    childs = children(parent);
    issum = false; isprod = false; ispow = false; isfun = false;

    if numel(childs) == 1
        if ~isequaln(childs,parent) % functions were removed
            isfun = true;
        end
        return
    end

    if numel(childs) == 2
        if isequaln(childs(1)^childs(2),parent) % pow
            ispow = true;
            return
        elseif isequaln(childs(1)/childs(2),parent) % div
            childs = parent;
            return
        end
    end

    if isequaln(prod(childs),parent) % prod
        isprod = true;
        return
    end

    if isequaln(sum(childs),parent) % sum
        issum = true;
        return
    end

    error('children2: Undefined behaviour for more than two children')

end