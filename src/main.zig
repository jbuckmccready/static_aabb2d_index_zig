const std = @import("std");
const testing = std.testing;

pub fn AABB(comptime T: type) type {
    return struct {
        min_x: T,
        min_y: T,
        max_x: T,
        max_y: T,

        const Self = @This();
        pub fn init(min_x: T, min_y: T, max_x: T, max_y: T) Self {
            return Self{
                .min_x = min_x,
                .min_y = min_y,
                .max_x = max_x,
                .max_y = max_y,
            };
        }
    };
}

pub fn StaticAABB2DIndex(comptime T: type) type {
    return struct {
        node_size: usize,
        num_items: usize,
        level_bounds: []usize,
        boxes: []AABB(T),
        indices: []usize,
        allocator: std.mem.Allocator,

        const Self = @This();

        fn consumeBuilder(builder: *StaticAABB2DIndexBuilder(T)) Self {
            const result = Self{
                .node_size = builder.node_size,
                .num_items = builder.num_items,
                .level_bounds = builder.level_bounds,
                .boxes = builder.boxes,
                .indices = builder.indices,
                .allocator = builder.allocator,
            };
            builder.level_bounds = &[_]usize{};
            builder.boxes = &[_]AABB(T){};
            builder.indices = &[_]usize{};
            return result;
        }

        pub fn deinit(self: Self) void {
            self.allocator.free(self.level_bounds);
            self.allocator.free(self.boxes);
            self.allocator.free(self.indices);
        }
    };
}

/// Error type for errors that may be returned in attempting to build the index.
pub const StaticAABB2DIndexBuildError = error{
    /// Error for the case when the number of items added does not match the size given when
    /// the builder was initialized.
    WrongItemCount,
    OutOfMemory,
};

pub fn StaticAABB2DIndexBuilder(comptime T: type) type {
    return struct {
        node_size: usize,
        num_items: usize,
        level_bounds: []usize,
        boxes: []AABB(T),
        indices: []usize,
        pos: usize,
        allocator: std.mem.Allocator,

        const Self = @This();

        pub fn init(allocator: std.mem.Allocator, num_items: usize, node_size: usize) !Self {
            if (num_items == 0) {
                // just return early, with no items added
                return Self{
                    .node_size = node_size,
                    .num_items = num_items,
                    .level_bounds = &[_]usize{},
                    .boxes = &[_]AABB(T){},
                    .indices = &[_]usize{},
                    .pos = 0,
                    .allocator = allocator,
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
            errdefer allocator.free(boxes);

            const indices = try allocator.alloc(usize, num_nodes);
            errdefer allocator.free(indices);
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
                .allocator = allocator,
            };
        }

        pub fn deinit(self: Self) void {
            self.allocator.free(self.level_bounds);
            self.allocator.free(self.boxes);
            self.allocator.free(self.indices);
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

            self.boxes[self.pos] = AABB(T).init(min_x, min_y, max_x, max_y);

            self.pos += 1;
        }

        pub fn build(self: *Self) StaticAABB2DIndexBuildError!StaticAABB2DIndex(T) {
            if (self.pos != self.num_items) {
                return StaticAABB2DIndexBuildError.WrongItemCount;
            }

            if (self.num_items == 0) {
                return StaticAABB2DIndex(T).consumeBuilder(self);
            }

            // calculate total bounds
            var min_x = self.boxes[0].min_x;
            var min_y = self.boxes[0].min_y;
            var max_x = self.boxes[0].max_x;
            var max_y = self.boxes[0].max_y;
            for (1..self.boxes.len) |i| {
                min_x = @min(min_x, self.boxes[i].min_x);
                min_y = @min(min_y, self.boxes[i].min_y);
                max_x = @min(max_x, self.boxes[i].max_x);
                max_y = @min(max_y, self.boxes[i].max_y);
            }
            // if number of items is less than node size then skip sorting since each node of boxes must
            // be fully scanned regardless and there is only one node
            if (self.num_items <= self.node_size) {
                self.indices[self.pos] = 0;
                // fill root box with total extents
                self.boxes[self.pos] = AABB(T).init(min_x, min_y, max_x, max_y);

                return StaticAABB2DIndex(T).consumeBuilder(self);
            }

            const width = max_x - min_x;
            const height = max_y - min_y;
            // hilbert max input value for x and y
            const hilbert_max: T = @floatFromInt(std.math.maxInt(u16));
            const two: T = @floatFromInt(2);
            // mapping the x and y coordinates of the center of the item boxes to values in the range
            // [0 -> n - 1] such that the min of the entire set of bounding boxes maps to 0 and the max
            // of the entire set of bounding boxes maps to n - 1 our 2d space is x: [0 -> n-1] and
            // y: [0 -> n-1], our 1d hilbert curve value space is d: [0 -> n^2 - 1]
            var hilbert_values = try self.allocator.alloc(u32, self.num_items);
            defer self.allocator.free(hilbert_values);

            for (0..self.num_items) |i| {
                const aabb = self.boxes[i];
                var x: u16 = 0;
                if (width != @as(T, 0)) {
                    x = @intFromFloat(hilbert_max * ((aabb.min_x + aabb.max_x) / two - min_x) / width);
                }
                var y: u16 = 0;
                if (height != @as(T, 0)) {
                    y = @intFromFloat(hilbert_max * ((aabb.min_y + aabb.max_y) / two - min_y) / height);
                }

                hilbert_values[i] = hilbertXYToIndex(x, y);
            }

            // sort items by their Hilbert value for constructing the tree
            sort(
                T,
                hilbert_values,
                self.boxes,
                self.indices,
                0,
                self.num_items - 1,
                self.node_size,
            );

            // generate nodes at each tree level, bottom-up
            var pos: usize = 0;
            for (0..self.level_bounds.len - 1) |i| {
                const end = self.level_bounds[i];

                while (pos < end) {
                    var node_min_x = std.math.floatMax(T);
                    var node_min_y = std.math.floatMax(T);
                    var node_max_x = std.math.floatMin(T);
                    var node_max_y = std.math.floatMin(T);
                    const node_index = pos;

                    // calculate bounding box for the new node
                    var j: usize = 0;
                    while (j < self.node_size and pos < end) {
                        const aabb = self.boxes[pos];
                        pos += 1;
                        node_min_x = @min(node_min_x, aabb.min_x);
                        node_min_y = @min(node_min_y, aabb.min_y);
                        node_max_x = @min(node_max_x, aabb.max_x);
                        node_max_y = @min(node_max_y, aabb.max_y);
                        j += 1;
                    }

                    // add the new node to the tree
                    self.indices[self.pos] = node_index;
                    self.boxes[self.pos] = AABB(T).init(node_min_x, node_min_y, node_max_x, node_max_y);
                    self.pos += 1;
                }
            }

            return StaticAABB2DIndex(T).consumeBuilder(self);
        }
    };
}

/// Maps 2d space to 1d hilbert curve space.
///
/// 2d space is `x_coord: [0 -> n-1]` and `y_coord: [0 -> n-1]`, 1d hilbert curve value space is
/// `d: [0 -> n^2 - 1]`, where n = 2^16, so `x_coord` and `y_coord` must be between 0 and
/// intMax(u16) (65535 or 2^16 - 1).
pub fn hilbertXYToIndex(x_coord: u16, y_coord: u16) u32 {
    const x: u32 = x_coord;
    const y: u32 = y_coord;
    // Fast Hilbert curve algorithm by http://threadlocalmutex.com/
    // Ported from C++ https://github.com/rawrunprotected/hilbert_curves (public domain)
    var a_1 = x ^ y;
    var b_1 = 0xFFFF ^ a_1;
    var c_1 = 0xFFFF ^ (x | y);
    var d_1 = x & (y ^ 0xFFFF);

    var a_2 = a_1 | (b_1 >> 1);
    var b_2 = (a_1 >> 1) ^ a_1;
    var c_2 = ((c_1 >> 1) ^ (b_1 & (d_1 >> 1))) ^ c_1;
    var d_2 = ((a_1 & (c_1 >> 1)) ^ (d_1 >> 1)) ^ d_1;

    a_1 = a_2;
    b_1 = b_2;
    c_1 = c_2;
    d_1 = d_2;
    a_2 = (a_1 & (a_1 >> 2)) ^ (b_1 & (b_1 >> 2));
    b_2 = (a_1 & (b_1 >> 2)) ^ (b_1 & ((a_1 ^ b_1) >> 2));
    c_2 ^= (a_1 & (c_1 >> 2)) ^ (b_1 & (d_1 >> 2));
    d_2 ^= (b_1 & (c_1 >> 2)) ^ ((a_1 ^ b_1) & (d_1 >> 2));

    a_1 = a_2;
    b_1 = b_2;
    c_1 = c_2;
    d_1 = d_2;
    a_2 = (a_1 & (a_1 >> 4)) ^ (b_1 & (b_1 >> 4));
    b_2 = (a_1 & (b_1 >> 4)) ^ (b_1 & ((a_1 ^ b_1) >> 4));
    c_2 ^= (a_1 & (c_1 >> 4)) ^ (b_1 & (d_1 >> 4));
    d_2 ^= (b_1 & (c_1 >> 4)) ^ ((a_1 ^ b_1) & (d_1 >> 4));

    a_1 = a_2;
    b_1 = b_2;
    c_1 = c_2;
    d_1 = d_2;
    c_2 ^= (a_1 & (c_1 >> 8)) ^ (b_1 & (d_1 >> 8));
    d_2 ^= (b_1 & (c_1 >> 8)) ^ ((a_1 ^ b_1) & (d_1 >> 8));

    a_1 = c_2 ^ (c_2 >> 1);
    b_1 = d_2 ^ (d_2 >> 1);

    var i_0 = x ^ y;
    var i_1 = b_1 | (0xFFFF ^ (i_0 | a_1));

    i_0 = (i_0 | (i_0 << 8)) & 0x00FF00FF;
    i_0 = (i_0 | (i_0 << 4)) & 0x0F0F0F0F;
    i_0 = (i_0 | (i_0 << 2)) & 0x33333333;
    i_0 = (i_0 | (i_0 << 1)) & 0x55555555;

    i_1 = (i_1 | (i_1 << 8)) & 0x00FF00FF;
    i_1 = (i_1 | (i_1 << 4)) & 0x0F0F0F0F;
    i_1 = (i_1 | (i_1 << 2)) & 0x33333333;
    i_1 = (i_1 | (i_1 << 1)) & 0x55555555;

    return (i_1 << 1) | i_0;
}

// modified quick sort that skips sorting boxes within the same node
fn sort(
    comptime T: type,
    values: []u32,
    boxes: []AABB(T),
    indices: []usize,
    left: usize,
    right: usize,
    node_size: usize,
) void {
    std.debug.assert(left <= right);
    if (left / node_size >= right / node_size) {
        // remaining to be sorted fits within the the same node, skip sorting further
        // since all boxes within a node must be visited when querying regardless
        return;
    }

    const mid = (left + right) / 2;
    const pivot = values[mid];
    var i = left -% 1;
    var j = right +% 1;

    while (true) {
        while (true) {
            i = i +% 1;
            if (values[i] >= pivot) {
                break;
            }
        }

        while (true) {
            j = j +% 1;
            if (values[j] <= pivot) {
                break;
            }
        }

        if (i >= j) {
            break;
        }

        swap(T, values, boxes, indices, i, j);
    }
}

fn swap(comptime T: type, values: []u32, boxes: []AABB(T), indices: []usize, i: usize, j: usize) void {
    std.mem.swap(u32, &values[i], &values[j]);
    std.mem.swap(AABB(T), &boxes[i], &boxes[j]);
    std.mem.swap(usize, &indices[i], &indices[j]);
}

test "wrong item count build error" {
    var builder = try StaticAABB2DIndexBuilder(f64).init(std.testing.allocator, 10, 16);
    defer builder.deinit();
    const spatial_index = builder.build();
    try std.testing.expectError(StaticAABB2DIndexBuildError.WrongItemCount, spatial_index);
}
