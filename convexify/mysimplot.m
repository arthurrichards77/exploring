function f_out = mysimplot(u)

f_in = u(1);
pos = u(2);
occSize = u(3:4);
occBox = u(5:8);
occ = reshape(u(9:end),occSize(1),occSize(2));

if f_in>0,
    f_out=f_in;
else
    f_out = figure;
end

% make reference object
occRef = imref2d(size(occ),occBox(1:2)',occBox(3:4)');

% show the map
figure(f_out)
clf
%imshow(occ,occRef)
imshow(~occ,occRef)
hold on
plot(50+2*pos,50+2*pos,'go')
pause(0.01)