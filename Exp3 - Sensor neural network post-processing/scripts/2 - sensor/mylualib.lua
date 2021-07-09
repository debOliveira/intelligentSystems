--
function copya(t)
local u = {}
for k, v in pairs(t) do
    u[k] = v
end
return u
end
--
function findv(t,v)
for i,j in pairs(t) do
    if j == v then return i end
    end
end
