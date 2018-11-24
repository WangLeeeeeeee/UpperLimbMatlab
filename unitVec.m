% 摘自：https://blog.csdn.net/libing403/article/details/78234640
%自定义函数，对向量单位化
function vector=unitVec(vec)
vector=vec/sqrt(sum(vec.^2));
end