function handle =circle(center,radius,NOP,style, fillStyle, varargin)
%---------------------------------------------------------------------------------------------
% H=CIRCLE(H, CENTER,RADIUS,NOP,STYLE) 
% This routine draws a circle with center defined as
% a vector CENTER, radius as a scaler RADIS. NOP is 
% the number of points on the circle. As to STYLE,
% use it the same way as you use the rountine PLOT.
% Since the handle of the object is returned, you
% use routine SET to get the best result.
%
%   Usage Examples,
%
%   circle([1,3],3,1000,':'); 
%   circle([2,4],2,1000,'--');
%
%   Zhenhai Wang <zhenhai@ieee.org>
%   Version 1.00
%   December, 2002
%---------------------------------------------------------------------------------------------

if (nargin <3),
 error('Please see help for INPUT DATA.');
elseif (nargin==3)
    style='b-';
end;

if nargin < 6
    handle.line = [];
    handle.fill = [];
else
    handle = varargin{1};
    if isempty(handle)
        handle.line = [];
        handle.fill = [];
    end
end

THETA=linspace(0,2*pi,NOP);
RHO=ones(1,NOP)*radius;
[X,Y] = pol2cart(THETA,RHO);
X=X+center(1);
Y=Y+center(2);

if isempty(handle.line)
    handle.line = plot(X,Y,'Color', style, 'linewidth', 3);
    hold on;
else
    set(handle.line, 'xdata', X, 'ydata', Y);
end

%% Fill in circle if specified
if length(fillStyle) == 1
    if fillStyle == -1
        return;
    end
end

if nargin > 4 
    if isempty(handle.fill)
        if(~isempty(fillStyle))
            handle.fill = fill(X, Y, fillStyle, 'EdgeColor', 'none');
            hold on;
        end
    else
        set(handle.fill, 'xdata', X, 'ydata', Y);
    end
    
end
