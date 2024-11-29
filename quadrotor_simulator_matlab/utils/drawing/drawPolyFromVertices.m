function drawPolyFromVertices(c,color,alpha)

k = convhull(c);
X = reshape(c(k',1), size(k'));
Y = reshape(c(k',2), size(k'));
Z = reshape(c(k',3), size(k'));
%plot3(X,Y,Z,color)
%fill3(X,Y,Z,color, 'FaceAlpha', 0.5)
surf(X,Y,Z,'FaceColor',color,'FaceAlpha',alpha,'EdgeColor','None');
end