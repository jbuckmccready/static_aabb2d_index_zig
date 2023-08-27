const std = @import("std");
const testing = std.testing;

pub fn AABB(comptime T: type) type {
    return struct {
        min_x: T,
        min_y: T,
        max_x: T,
        max_y: T,
    };
}

pub fn StaticAABB2DIndex(comptime T: type) type {
    return struct {
        node_size: usize,
        num_items: usize,
        level_bounds: []usize,
        boxes: []AABB(T),
        indices: []usize,

        const Self = @This();

        pub fn init(num_items: usize, node_size: usize) Self {
            _ = node_size;
            _ = num_items;
            @panic("todo!");
        }
    };
}

pub const StaticAABB2DIndexBuildError = error{
    ItemCountError,
};

pub fn StaticAABB2DIndexBuilder(comptime T: type) type {
    return struct {
        node_size: usize,
        num_items: usize,
        level_bounds: []usize,
        boxes: []AABB(T),
        indices: []usize,
        pos: usize,

        const Self = @This();

        pub fn init(allocator: std.mem.Allocator, num_items: usize, node_size: usize) Self {
            if (num_items == 0) {
                // just return early, with no items added
                return Self{
                    .node_size = node_size,
                    .num_items = num_items,
                    .level_bounds = &[_]usize{},
                    .boxes = &[_]AABB{},
                    .indices = &[_]usize{},
                    .pos = 0,
                };
            }

            const node_sz = std.math.clamp(node_size, 2, 65535);

            var n = num_items;
            var num_nodes = num_items;
            var level_bounds_builder = std.ArrayList(usize).init(allocator);

            // calculate the total number of nodes in the R-tree to allocate space for
            // and the index of each tree level (level_bounds, used in search later)

            // ensure level_bounds freed if error occurs while building it
            errdefer level_bounds_builder.deinit();
            while (true) {
                const numer: f64 = @floatFromInt(n);
                const denom: f64 = @floatFromInt(node_sz);
                n = @intFromFloat(@ceil(numer / denom));
                num_nodes += n;
                try level_bounds_builder.append(num_nodes);
                if (n == 1) {
                    break;
                }
            }
            const level_bounds = try level_bounds_builder.toOwnedSlice();

            // unitialized array to hold AABB
            const boxes = try allocator.alloc(AABB(T), num_nodes);
            errdefer boxes.deinit();

            const indices = try allocator.alloc(usize, num_nodes);
            errdefer indices.deinit();
            for (0..num_nodes) |i| {
                indices[i] = i;
            }

            return Self{
                .node_size = node_sz,
                .num_items = num_items,
                .level_bounds = level_bounds,
                .boxes = boxes,
                .indices = indices,
                .pos = 0,
            };
        }

        /// Add an axis aligned bounding box with the extent points (`min_x`, `min_y`),
        /// (`max_x`, `max_y`) to the index.
        ///
        /// `min_x <= max_x` and `min_y <= max_y` are std.debug.asserted. If an invalid box is added
        /// it may lead to a panic or unexpected behavior.
        pub fn add(self: *Self, min_x: T, min_y: T, max_x: T, max_y: T) void {
            // catch adding past num_items (error will be returned when build is called)
            if (self.pos >= self.num_items) {
                self.pos += 1;
                return;
            }

            std.debug.assert(min_x <= max_x);
            std.debug.assert(min_y <= max_y);

            self.boxes[self.pos] = AABB(T){
                .min_x = min_x,
                .min_y = min_y,
                .max_x = max_x,
                .max_y = max_y,
            };

            self.pos += 1;
        }

        pub fn build(self: *Self) StaticAABB2DIndexBuildError!StaticAABB2DIndex(T) {
            _ = self;
        }
    };
}

test "basic add functionality" {
    var builder = StaticAABB2DIndexBuilder(f64).init(std.testing.allocator, 10, 16);
    const spatial_index = builder.build();
    _ = spatial_index;
}
