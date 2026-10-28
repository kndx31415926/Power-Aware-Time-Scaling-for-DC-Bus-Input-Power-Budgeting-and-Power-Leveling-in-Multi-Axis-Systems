function [robot_out, pay_vec] = spinn3d_inject_payload_last_body(robot_in, pay)
% spinn3d_inject_payload_last_body

%

%

    % Work on a private tree so sequential method evaluations cannot leak a
    % payload modification into the next method through a shared handle.
    try
        robot_out = copy(robot_in);
    catch MEcopy
        error('spinn3d_inject_payload_last_body:CopyFailed', ...
            ['A private rigidBodyTree copy is required before payload ', ...
             'injection; refusing to mutate the caller-owned robot: %s'], ...
            MEcopy.message);
    end
    pay_vec   = zeros(1,10);
    if nargin < 2 || isempty(pay)
        return;
    end

    if isnumeric(pay)
        v = double(pay(:)).';
        if numel(v) < 10, v = [v, zeros(1,10-numel(v))]; end
        m  = v(1);
        com = v(2:4);
        I6 = v(5:10);
        about = 'com';
        mode  = 'replace';
    elseif isstruct(pay)
        m     = getfield_def(pay,'mass',0);
        com   = getfield_def(pay,'com',[0 0 0]);
        I6    = getfield_def(pay,'inertia',[0 0 0 0 0 0]);
        about = lower(getfield_def(pay,'about','com'));
        mode  = lower(getfield_def(pay,'mode','replace'));
    else
        return;
    end

    pay_vec = [double(m), double(com(:)).', double(I6(:)).'];

    try
        body_name = robot_out.BodyNames{end};
        b = copy(getBody(robot_out,body_name));
        m0 = double(b.Mass);
        c0 = double(b.CenterOfMass(:)).';
        I0 = I6ToMat(double(b.Inertia(:)).');

        Ic = I6ToMat(double(I6(:)).');
        if strcmpi(about,'com')
            r  = double(com(:)); r = r(:);
            I1 = Ic + double(m) * ((r.'*r)*eye(3) - (r*r.'));
        else
            I1 = Ic;
        end

        switch mode
            case 'add'
                m_new  = m0 + m;
                if m_new <= 0
                    c_new = [0 0 0];
                else
                    c_new = (m0*c0 + m*com(:).') / m_new;
                end
                I_new = I0 + I1;
            otherwise % 'replace'
                m_new = m;
                c_new = com(:).';
                I_new = I1;
        end

        

        b.Mass         = m_new;
        b.CenterOfMass = c_new;
        b.Inertia      = ImatTo6(I_new).';
        replaceBody(robot_out,body_name,b);

        b_check = getBody(robot_out,body_name);

        
    catch ME
        error('spinn3d_inject_payload_last_body:InjectionFailed', ...
            'Payload injection failed and was not ignored: %s',ME.message);
    end
end

% ===== helpers =====
function I = I6ToMat(v6)

    v6 = double(v6(:)).';
    Ixx=v6(1); Iyy=v6(2); Izz=v6(3);
    Iyz=v6(4); Ixz=v6(5); Ixy=v6(6);
    I = [ Ixx Ixy Ixz;
          Ixy Iyy Iyz;
          Ixz Iyz Izz ];
end

function v6 = ImatTo6(I)

    v6 = [ I(1,1), I(2,2), I(3,3), I(2,3), I(1,3), I(1,2) ];
end

function v = getfield_def(S,fn,def)
    if isstruct(S) && isfield(S,fn) && ~isempty(S.(fn))
        v = S.(fn);
    else
        v = def;
    end
end
