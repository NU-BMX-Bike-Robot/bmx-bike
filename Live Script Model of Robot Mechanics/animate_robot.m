%% animate_robot.m
%
% Description:
%   Animates the robot according to a list of configurations over time.
%   
% Inputs:
%   q_list: 2xN list of configurations q, where q = [x_cart; theta_pend];
%   params: a struct with many elements, generated by calling init_params.m
%   varargin: optional name-value pair arguments:
%       'trace_cart_com': (default: false), if true, plots a tracer on the
%           cart's center of mass (CoM)
%       'trace_pend_com': (default: false), if true, plots a tracer on the
%           pendulum's center of mass (CoM)
%       'trace_pend_tip': (default: false), if true, plots a tracer on the
%           pendulum's tip
%       'video': (default: false), if true, animation is written to file.
%
% Outputs:
%   none

function animate_robot(q_list,params,varargin)

% Parse input arguments
% Note: a simple robot animation function doesn't need this, but I want to
% write extensible code, so I'm using "varargin" which requires input
% parsing. See the reference below:
%
% https://people.umass.edu/whopper/posts/better-matlab-functions-with-the-inputparser-class/

% Step 1: instantiate an inputParser:
p = inputParser;

% Step 2: create the parsing schema:
%   2a: required inputs:
addRequired(p,'cart_pend_config', ...
    @(q_list) isnumeric(q_list) && size(q_list,1)==5);
addRequired(p,'cart_pend_params', ...
    @(params) ~isempty(params));
%   2b: optional inputs:
%       optional name-value pairs to trace different parts of the robot:
addParameter(p, 'trace_cart_com', false);
addParameter(p, 'trace_pend_com', false);
addParameter(p, 'trace_pend_tip', false);
addParameter(p, 'video', false);


% Step 3: parse the inputs:
parse(p, q_list, params, varargin{:});

fig_handle = figure('Renderer', 'painters', 'Position', [10 10 900 600]);

if (p.Results.trace_cart_com || p.Results.trace_pend_com ...
        || p.Results.trace_pend_tip)
    tracing = true;
else
    tracing = false;
end
    
    if p.Results.video
        v = VideoWriter('bikeanim.avi');
        open(v);
    end
    
    if tracing
        cart.curr.com.x = [];
        cart.curr.com.y = [];
        
        pend.curr.com.x = [];
        pend.curr.com.y = [];
        
        pend.curr.tip.x = [];
        pend.curr.tip.y = [];
    end
    
    for i = 1:size(q_list,2)
        plot_robot(q_list(:,i),params,'new_fig',false);
        
        if tracing
            FK = fwd_kin(q_list(:,i),params);

            % append (x,y) location of cart CoM:
            cart.curr.com.x = [cart.curr.com.x, FK(1,1)];
            cart.curr.com.y = [cart.curr.com.y, FK(2,1)];

            % append (x,y) location of pendulum CoM:
            pend.curr.com.x = [pend.curr.com.x,FK(1,2)];
            pend.curr.com.y = [pend.curr.com.y,FK(2,2)];

            % append (x,y) location of pendulum tip:
            pend.curr.tip.x = [pend.curr.tip.x,FK(1,3)];
            pend.curr.tip.y = [pend.curr.tip.y,FK(2,3)];
            
            if p.Results.trace_cart_com
                hold on;
                plot(cart.curr.com.x,cart.curr.com.y,'o-',...
                    'Color',params.viz.colors.tracers.cart_com,...
                    'MarkerSize',3,'LineWidth',2,...
                    'MarkerFaceColor',params.viz.colors.tracers.cart_com,...
                    'MarkerEdgeColor',params.viz.colors.tracers.cart_com);
                hold off;
            end
            if p.Results.trace_pend_com
                hold on;
                plot(pend.curr.com.x,pend.curr.com.y,'o-',...
                    'Color',params.viz.colors.tracers.pend_com,...
                    'MarkerSize',3,'LineWidth',2,...
                    'MarkerFaceColor',params.viz.colors.tracers.pend_com,...
                    'MarkerEdgeColor',params.viz.colors.tracers.pend_com);
                hold off;
            end
            if p.Results.trace_pend_tip
                hold on;
                plot(pend.curr.tip.x,pend.curr.tip.y,'o-',...
                    'Color',params.viz.colors.tracers.pend_tip,...
                    'MarkerSize',3,'LineWidth',2,...
                    'MarkerFaceColor',params.viz.colors.tracers.pend_tip,...
                    'MarkerEdgeColor',params.viz.colors.tracers.pend_tip);
                hold off;
            end
        end
        
        if p.Results.video
            M(i) = getframe(fig_handle);
            writeVideo(v,M(i));
        end
    end
    
    if p.Results.video
        close(v);
%         movie(gcf,M); % comment this out if you don't want to see replay
    end
end