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

        pub fn overlaps(self: Self, min_x: T, min_y: T, max_x: T, max_y: T) bool {
            if (self.max_x < min_x or
                self.max_y < min_y or
                self.min_x > max_x or
                self.min_y > max_y)
            {
                return false;
            }

            return true;
        }
    };
}

pub fn StaticAABB2DIndex(comptime T: type) type {
    return struct {
        /// Node size defined  when the index was built.
        node_size: usize,
        /// Total number of items in the index.
        num_items: usize,
        /// Level bounds for traversing the index tree.
        level_bounds: []usize,
        /// All the AABB boxes (items and tree containers).
        boxes: []AABB(T),
        /// Indices to map from a `boxes` index back to the original index position the item was
        /// added. Note there ar more indices than `num_items`, the first `num_items` items in the
        /// slice are associated with the `boxes`.
        indices: []usize,
        /// Allocator used to manage the memory of the spatial index.
        allocator: std.mem.Allocator,

        const Self = @This();

        /// Gets the total bounds of all the items that were added to the index or null if the index
        /// had no items added in construction (item count is 0).
        pub fn bounds(self: Self) ?AABB(T) {
            if (self.boxes.len == 0) {
                return null;
            }
            return self.boxes[self.boxes.len - 1];
        }

        fn collectingVisitor(context: *std.ArrayList(usize), index: usize) error{OutOfMemory}!bool {
            try context.append(index);
            return true;
        }
        pub fn query(
            self: Self,
            min_x: T,
            min_y: T,
            max_x: T,
            max_y: T,
            allocator: std.mem.Allocator,
        ) error{OutOfMemory}!std.ArrayList(usize) {
            var result = std.ArrayList(usize).init(allocator);
            errdefer result.deinit();
            var stack = std.ArrayList(usize).init(allocator);
            defer stack.deinit();

            self.visitQueryWithStack(
                min_x,
                min_y,
                max_x,
                max_y,
                &result,
                collectingVisitor,
                &stack,
            ) catch return error.OutOfMemory;

            return result;
        }

        /// Perform query with a visitor function and using an existing stack for traversal, if the
        /// function returns true the query continues, otherwise the query stops.
        pub fn visitQueryWithStack(
            self: Self,
            min_x: T,
            min_y: T,
            max_x: T,
            max_y: T,
            context: anytype,
            comptime visitorFn: fn (@TypeOf(context), index: usize) anyerror!bool,
            stack: *std.ArrayList(usize),
        ) !void {
            var node_index = self.boxes.len - 1;
            var level = self.level_bounds.len - 1;
            stack.clearRetainingCapacity();

            while (true) {
                const end = @min(node_index + self.node_size, self.level_bounds[level]);
                for (node_index..end) |pos| {
                    const aabb = self.boxes[pos];
                    if (!aabb.overlaps(min_x, min_y, max_x, max_y)) {
                        // no overlap
                        continue;
                    }

                    const index = self.indices[pos];
                    if (node_index < self.num_items) {
                        if (!try visitorFn(context, index)) {
                            return;
                        }
                    } else {
                        try stack.append(index);
                        try stack.append(level - 1);
                    }
                }

                if (stack.items.len > 1) {
                    level = stack.pop();
                    node_index = stack.pop();
                } else {
                    return;
                }
            }
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
    /// Error occurs when allocator fails to allocate when building the index.
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

        pub fn init(allocator: std.mem.Allocator, num_items: usize) error{OutOfMemory}!Self {
            return try Self.initWithNodeSize(allocator, num_items, 16);
        }

        pub fn initWithNodeSize(
            allocator: std.mem.Allocator,
            num_items: usize,
            node_size: usize,
        ) error{OutOfMemory}!Self {
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
            // ensure level_bounds freed if error occurs while building it
            errdefer level_bounds_builder.deinit();

            try level_bounds_builder.append(n);
            // calculate the total number of nodes in the R-tree to allocate space for
            // and the index of each tree level (level_bounds, used in search later)
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
            errdefer allocator.free(level_bounds);

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

        const typesSupportedMsg = "only int and float types are supported";
        fn u16ToNumT(input: u16) T {
            return switch (@typeInfo(T)) {
                .Float => @floatFromInt(input),
                .Int => @intCast(input),
                else => @compileError(typesSupportedMsg),
            };
        }

        fn numTToU16(num: T) u16 {
            return switch (@typeInfo(T)) {
                .Float => @intFromFloat(num),
                .Int => @intCast(num),
                else => @compileError(typesSupportedMsg),
            };
        }

        fn numTDiv(numerator: T, denominator: T) T {
            return switch (@typeInfo(T)) {
                .Float => numerator / denominator,
                .Int => @divTrunc(numerator, denominator),
                else => @compileError(typesSupportedMsg),
            };
        }

        const numTMinVal = switch (@typeInfo(T)) {
            .Float => -std.math.floatMax(T),
            .Int => @as(T, std.math.minInt(T)),
            else => @compileError(typesSupportedMsg),
        };

        const numTMaxVal = switch (@typeInfo(T)) {
            .Float => std.math.floatMax(T),
            .Int => @as(T, std.math.maxInt(T)),
            else => @compileError(typesSupportedMsg),
        };

        pub fn build(self: *Self) StaticAABB2DIndexBuildError!StaticAABB2DIndex(T) {
            if (self.pos != self.num_items) {
                return StaticAABB2DIndexBuildError.WrongItemCount;
            }

            if (self.num_items == 0) {
                return self.intoIndex();
            }

            // calculate total bounds
            var min_x = self.boxes[0].min_x;
            var min_y = self.boxes[0].min_y;
            var max_x = self.boxes[0].max_x;
            var max_y = self.boxes[0].max_y;
            for (1..self.num_items) |i| {
                min_x = @min(min_x, self.boxes[i].min_x);
                min_y = @min(min_y, self.boxes[i].min_y);
                max_x = @max(max_x, self.boxes[i].max_x);
                max_y = @max(max_y, self.boxes[i].max_y);
            }
            // if number of items is less than node size then skip sorting since each node of boxes must
            // be fully scanned regardless and there is only one node
            if (self.num_items <= self.node_size) {
                self.indices[self.pos] = 0;
                // fill root box with total extents
                self.boxes[self.pos] = AABB(T).init(min_x, min_y, max_x, max_y);

                return self.intoIndex();
            }

            const width = max_x - min_x;
            const height = max_y - min_y;
            // hilbert max input value for x and y
            const hilbert_max: T = u16ToNumT(std.math.maxInt(u16));
            const two: T = u16ToNumT(2);
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
                    const x_mid = numTDiv((aabb.min_x + aabb.max_x), two);
                    x = numTToU16(hilbert_max * numTDiv(x_mid - min_x, width));
                }
                var y: u16 = 0;
                if (height != @as(T, 0)) {
                    const y_mid = numTDiv((aabb.min_y + aabb.max_y), two);
                    y = numTToU16(hilbert_max * numTDiv((y_mid - min_y), height));
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
                    var node_min_x = numTMaxVal;
                    var node_min_y = numTMaxVal;
                    var node_max_x = numTMinVal;
                    var node_max_y = numTMinVal;
                    const node_index = pos;

                    // calculate bounding box for the new node
                    var j: usize = 0;
                    while (j < self.node_size and pos < end) {
                        const aabb = self.boxes[pos];
                        pos += 1;
                        node_min_x = @min(node_min_x, aabb.min_x);
                        node_min_y = @min(node_min_y, aabb.min_y);
                        node_max_x = @max(node_max_x, aabb.max_x);
                        node_max_y = @max(node_max_y, aabb.max_y);
                        j += 1;
                    }

                    // add the new node to the tree
                    self.indices[self.pos] = node_index;
                    self.boxes[self.pos] = AABB(T).init(node_min_x, node_min_y, node_max_x, node_max_y);
                    self.pos += 1;
                }
            }

            return self.intoIndex();
        }

        /// Helper function to construct an owned index with builder data (consuming the builder).
        fn intoIndex(self: *Self) StaticAABB2DIndex(T) {
            const result = StaticAABB2DIndex(T){
                .node_size = self.node_size,
                .num_items = self.num_items,
                .level_bounds = self.level_bounds,
                .boxes = self.boxes,
                .indices = self.indices,
                .allocator = self.allocator,
            };
            self.level_bounds = &[_]usize{};
            self.boxes = &[_]AABB(T){};
            self.indices = &[_]usize{};
            return result;
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
            j = j -% 1;
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
    var builder = try StaticAABB2DIndexBuilder(f64).initWithNodeSize(std.testing.allocator, 2, 16);
    defer builder.deinit();
    var failed_build = builder.build();
    try std.testing.expectError(StaticAABB2DIndexBuildError.WrongItemCount, failed_build);
    builder.add(0, 0, 1, 1);
    failed_build = builder.build();
    try std.testing.expectError(StaticAABB2DIndexBuildError.WrongItemCount, failed_build);
    builder.add(1, 1, 3, 3);
    builder.add(-1, 1, 3, 3);
    failed_build = builder.build();
    try std.testing.expectError(StaticAABB2DIndexBuildError.WrongItemCount, failed_build);
}

test "bounds null when 0 items" {
    var builder = try StaticAABB2DIndexBuilder(f64).initWithNodeSize(std.testing.allocator, 0, 16);
    defer builder.deinit();
    const spatial_index = try builder.build();
    defer spatial_index.deinit();
    try std.testing.expect(spatial_index.bounds() == null);
}

test "bounds values when not 0 items" {
    var builder = try StaticAABB2DIndexBuilder(f64).initWithNodeSize(std.testing.allocator, 2, 16);
    defer builder.deinit();
    builder.add(0, 0, 1, 1);
    builder.add(1, 1, 3, 3);
    const spatial_index = try builder.build();
    defer spatial_index.deinit();
    try std.testing.expectEqual(AABB(f64).init(0, 0, 3, 3), spatial_index.bounds().?);
}

test "expected node size" {
    var builder = try StaticAABB2DIndexBuilder(f64).initWithNodeSize(std.testing.allocator, 2, 16);
    defer builder.deinit();
    builder.add(0, 0, 1, 1);
    builder.add(1, 1, 3, 3);
    const spatial_index = try builder.build();
    defer spatial_index.deinit();
    try std.testing.expectEqual(@as(usize, 16), spatial_index.node_size);
}

test "0 expected number of items" {
    var builder = try StaticAABB2DIndexBuilder(f64).initWithNodeSize(std.testing.allocator, 0, 16);
    defer builder.deinit();
    const spatial_index = try builder.build();
    defer spatial_index.deinit();
    try std.testing.expectEqual(@as(usize, 0), spatial_index.num_items);
}

test "expected number of items" {
    var builder = try StaticAABB2DIndexBuilder(f64).initWithNodeSize(std.testing.allocator, 2, 16);
    defer builder.deinit();
    builder.add(0, 0, 1, 1);
    builder.add(1, 1, 3, 3);
    const spatial_index = try builder.build();
    defer spatial_index.deinit();
    try std.testing.expectEqual(@as(usize, 2), spatial_index.num_items);
}

fn createTestData(comptime T: type) *const [400]T {
    return &[_]T{
        8,  62, 11, 66, 57, 17, 57, 19, 76, 26, 79, 29, 36, 56, 38, 56, 92, 77, 96, 80, 87, 70, 90,
        74, 43, 41, 47, 43, 0,  58, 2,  62, 76, 86, 80, 89, 27, 13, 27, 15, 71, 63, 75, 67, 25, 2,
        27, 2,  87, 6,  88, 6,  22, 90, 23, 93, 22, 89, 22, 93, 57, 11, 61, 13, 61, 55, 63, 56, 17,
        85, 21, 87, 33, 43, 37, 43, 6,  1,  7,  3,  80, 87, 80, 87, 23, 50, 26, 52, 58, 89, 58, 89,
        12, 30, 15, 34, 32, 58, 36, 61, 41, 84, 44, 87, 44, 18, 44, 19, 13, 63, 15, 67, 52, 70, 54,
        74, 57, 59, 58, 59, 17, 90, 20, 92, 48, 53, 52, 56, 92, 68, 92, 72, 26, 52, 30, 52, 56, 23,
        57, 26, 88, 48, 88, 48, 66, 13, 67, 15, 7,  82, 8,  86, 46, 68, 50, 68, 37, 33, 38, 36, 6,
        15, 8,  18, 85, 36, 89, 38, 82, 45, 84, 48, 12, 2,  16, 3,  26, 15, 26, 16, 55, 23, 59, 26,
        76, 37, 79, 39, 86, 74, 90, 77, 16, 75, 18, 78, 44, 18, 45, 21, 52, 67, 54, 71, 59, 78, 62,
        78, 24, 5,  24, 8,  64, 80, 64, 83, 66, 55, 70, 55, 0,  17, 2,  19, 15, 71, 18, 74, 87, 57,
        87, 59, 6,  34, 7,  37, 34, 30, 37, 32, 51, 19, 53, 19, 72, 51, 73, 55, 29, 45, 30, 45, 94,
        94, 96, 95, 7,  22, 11, 24, 86, 45, 87, 48, 33, 62, 34, 65, 18, 10, 21, 14, 64, 66, 67, 67,
        64, 25, 65, 28, 27, 4,  31, 6,  84, 4,  85, 5,  48, 80, 50, 81, 1,  61, 3,  61, 71, 89, 74,
        92, 40, 42, 43, 43, 27, 64, 28, 66, 46, 26, 50, 26, 53, 83, 57, 87, 14, 75, 15, 79, 31, 45,
        34, 45, 89, 84, 92, 88, 84, 51, 85, 53, 67, 87, 67, 89, 39, 26, 43, 27, 47, 61, 47, 63, 23,
        49, 25, 53, 12, 3,  14, 5,  16, 50, 19, 53, 63, 80, 64, 84, 22, 63, 22, 64, 26, 66, 29, 66,
        2,  15, 3,  15, 74, 77, 77, 79, 64, 11, 68, 11, 38, 4,  39, 8,  83, 73, 87, 77, 85, 52, 89,
        56, 74, 60, 76, 63, 62, 66, 65, 67,
    };
}

fn aabbFromTestData(comptime T: type, data: []const T) !std.ArrayList(T) {
    var result = std.ArrayList(T).init(std.testing.allocator);
    errdefer result.deinit();
    var i: usize = 0;
    while (i < data.len) : (i += 4) {
        try result.append(AABB(T).init(data[i], data[i + 1], data[i + 2], data[i + 3]));
    }

    return result;
}

fn createIndexFromData(comptime T: type, data: []const T) !StaticAABB2DIndex(T) {
    var builder = try StaticAABB2DIndexBuilder(T).init(std.testing.allocator, data.len / 4);
    var i: usize = 0;
    while (i < data.len) : (i += 4) {
        builder.add(data[i], data[i + 1], data[i + 2], data[i + 3]);
    }

    return builder.build();
}

fn createTestIndex(comptime T: type) !StaticAABB2DIndex(T) {
    return createIndexFromData(T, createTestData(T));
}

fn createSmallTestIndex(comptime T: type) !StaticAABB2DIndex(T) {
    const item_count = 14;
    createIndexFromData(T, createTestData(T)[0 .. 4 * item_count]);
}

test "building from zeroes" {
    {
        // f64 boxes
        const item_count = 50;
        var data = try std.testing.allocator.alloc(f64, 4 * item_count);
        defer std.testing.allocator.free(data);
        for (data) |*d| {
            d.* = 0;
        }

        const index = try createIndexFromData(f64, data);
        defer index.deinit();
        const query_results = try index.query(-1, -1, 1, 1, std.testing.allocator);
        defer query_results.deinit();
        std.sort.pdq(usize, query_results.items, {}, std.sort.asc(usize));
        var expected = try std.testing.allocator.alloc(usize, data.len / 4);
        defer std.testing.allocator.free(expected);
        for (0..expected.len) |i| {
            expected[i] = i;
        }

        try std.testing.expectEqualSlices(usize, expected, query_results.items);
    }

    {
        // i32 boxes
        const item_count = 50;
        var data = try std.testing.allocator.alloc(i32, 4 * item_count);
        defer std.testing.allocator.free(data);
        for (data) |*d| {
            d.* = 0;
        }

        const index = try createIndexFromData(i32, data);
        defer index.deinit();
        const query_results = try index.query(-1, -1, 1, 1, std.testing.allocator);
        defer query_results.deinit();
        std.sort.pdq(usize, query_results.items, {}, std.sort.asc(usize));
        var expected = try std.testing.allocator.alloc(usize, data.len / 4);
        defer std.testing.allocator.free(expected);
        for (0..expected.len) |i| {
            expected[i] = i;
        }

        try std.testing.expectEqualSlices(usize, expected, query_results.items);
    }
}

test "basic query" {
    const i32_index = try createTestIndex(i32);
    defer i32_index.deinit();
    var i32_results = try i32_index.query(40, 40, 60, 60, std.testing.allocator);
    defer i32_results.deinit();
    std.sort.insertion(usize, i32_results.items, {}, std.sort.asc(usize));

    const f64_index = try createTestIndex(f64);
    defer f64_index.deinit();
    var f64_results = try f64_index.query(40, 40, 60, 60, std.testing.allocator);
    defer f64_results.deinit();
    std.sort.insertion(usize, f64_results.items, {}, std.sort.asc(usize));
}
