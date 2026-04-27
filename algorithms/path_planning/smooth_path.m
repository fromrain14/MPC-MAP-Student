function [new_path] = smooth_path(old_path)

if isempty(old_path) || size(old_path,1)<3
    new_path = old_path;
    return;
end

alpha = 0.5;    % Vaha puvodnich bodu
beta = 0.25;    % Vaha vyhlazovani
n_iter = 500;   % Pocet opakovani algoritmu

x = old_path;
y = old_path;
n = size(y,1);

% iterativni update
for k=1:n_iter
    for i=2:n-1
        y(i,:) = y(i,:)+alpha*(x(i,:)-y(i,:))+beta*(y(i-1,:)+y(i+1,:)-2*y(i,:));
    end
end

new_path = y;
end