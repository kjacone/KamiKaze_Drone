-- Resolves a Docker container_id to its human-readable name by
-- reading Docker's own config.v2.json metadata file for that container.
-- Requires /var/lib/docker/containers mounted read-only (already the case
-- in fluent-bit's compose service).

function add_container_name(tag, timestamp, record)
    local id = record["container_id"]
    if id == nil then
        return 0, timestamp, record
    end

    local path = "/var/lib/docker/containers/" .. id .. "/config.v2.json"
    local f = io.open(path, "r")
    if f == nil then
        return 0, timestamp, record
    end

    local content = f:read("*a")
    f:close()

    -- config.v2.json stores the name as "Name":"/kamikaze_drone"
    local name = content:match('"Name":"/([^"]+)"')
    if name ~= nil then
        record["container_name"] = name
    else
        record["container_name"] = id
    end

    return 1, timestamp, record
end