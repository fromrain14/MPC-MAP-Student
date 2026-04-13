function [new_particles] = resample_particles(particles, weights)

N=size(particles, 1);
new_particles=zeros(N, size(particles, 2));

% nahodny start v intervalu (0, 1/N)
r=rand()/N;
c=weights(1);
i=1;

for k=1:N
    u=r+(k-1)/N;
    while u>c && i<N
        i=i+1;
        c=c+weights(i);
    end
    new_particles(k,:)=particles(i,:);
end
end