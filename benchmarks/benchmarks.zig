const std = @import("std");
const time = std.time;
const lib = @import("static_aabb2d_index");

fn printTiming(ns: f64) void {
    if (ns < 1000) {
        std.debug.print("{d:.0} ns/op\n", .{ns});
        return;
    }

    const us = ns / 1000;
    if (us < 1000) {
        std.debug.print("{d:.3} us/op\n", .{us});
        return;
    }

    const ms = us / 1000;
    if (ms < 1000) {
        std.debug.print("{d:.3} ms/op\n", .{ms});
        return;
    }

    const s = ms / 1000;
    if (s < 1000) {
        std.debug.print("{d:.3} s/op\n", .{s});
        return;
    }
}

const bench_cap_seconds = 1;
const bench_cap = time.ns_per_s * bench_cap_seconds;

pub fn bench(comptime name: []const u8, F: anytype, args: anytype) !void {
    var timer = try time.Timer.start();

    var loops: usize = 0;
    while (timer.read() < bench_cap) : (loops += 1) {
        _ = @call(.auto, F, args);

        if (loops > 100000) {
            break;
        }
    }

    const ns: f64 = @floatFromInt(timer.lap() / loops);

    const mgn = std.math.log10(loops);
    var loop_mgn: usize = 10;
    var i: usize = 0;
    while (i < mgn) : (i += 1) {
        loop_mgn *= 10;
    }

    std.debug.print("{s}: ~{d} loops\n   ", .{ name, loop_mgn });
    printTiming(ns);
}

const Point = struct { x: f64, y: f64 };

const RADIUS: f64 = 100.0;
// const QUERYCOUNT: usize = 100;

fn createPointsOnCircle(count: usize, allocator: std.mem.Allocator) ![]Point {
    const arr = try allocator.alloc(Point, count);
    for (0..count) |i| {
        const angle = std.math.tau * @as(f64, @floatFromInt(i)) / @as(f64, @floatFromInt(count));
        arr[i] = Point{ .x = RADIUS * std.math.cos(angle), .y = RADIUS * std.math.sin(angle) };
    }

    return arr;
}

fn minMax(a: f64, b: f64) struct { min: f64, max: f64 } {
    if (a < b) {
        return .{ .min = a, .max = b };
    }

    return .{ .min = b, .max = a };
}

fn createBoxesFromPointPairs(points: []Point, allocator: std.mem.Allocator) ![]lib.AABB(f64) {
    const arr = try allocator.alloc(lib.AABB(f64), points.len);
    for (1..points.len) |j| {
        const i = j - 1;
        const pt1 = points[i];
        const pt2 = points[j];
        const xs = minMax(pt1.x, pt2.x);
        const ys = minMax(pt1.y, pt2.y);
        arr[i] = lib.AABB(f64).init(xs.min, ys.min, xs.max, ys.max);
    }

    const last_pt = points[points.len - 1];
    const first_pt = points[0];
    const last_xs = minMax(last_pt.x, first_pt.x);
    const last_ys = minMax(last_pt.y, first_pt.y);
    arr[arr.len - 1] = lib.AABB(f64).init(last_xs.min, last_ys.min, last_xs.max, last_ys.max);
    return arr;
}

fn createIndexFromBoxes(boxes: []lib.AABB(f64), allocator: std.mem.Allocator) !lib.StaticAABB2DIndex(f64) {
    var b = try lib.StaticAABB2DIndexBuilder(f64).init(allocator, boxes.len);
    for (boxes) |box| {
        b.add(box.min_x, box.min_y, box.max_x, box.max_y);
    }

    return try b.build();
}

fn createIndexFromBoxesAndFree(boxes: []lib.AABB(f64), allocator: std.mem.Allocator) void {
    const index = createIndexFromBoxes(boxes, allocator) catch @panic("alloc failed");
    defer index.deinit();
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const gpa_allocator = gpa.allocator();
    defer _ = gpa.deinit();

    const test_input_allocator = std.heap.c_allocator;

    var arena = std.heap.ArenaAllocator.init(std.heap.c_allocator);
    defer arena.deinit();
    const arena_allocator = arena.allocator();

    // const points100 = try createPointsOnCircle(100, test_input_allocator);
    // defer test_input_allocator.free(points100);
    // const boxes100 = try createBoxesFromPointPairs(points100, test_input_allocator);
    // defer test_input_allocator.free(boxes100);

    // try bench("create index 100 boxes gpa", createIndexFromBoxesAndFree, .{ boxes100, gpa_allocator });
    // try bench("create index 100 boxes arena", createIndexFromBoxesAndFree, .{ boxes100, arena_allocator });
    // try bench("create index 100 boxes c alloc", createIndexFromBoxesAndFree, .{ boxes100, std.heap.c_allocator });

    // const points1000 = try createPointsOnCircle(1000, test_input_allocator);
    // defer test_input_allocator.free(points1000);
    // const boxes1000 = try createBoxesFromPointPairs(points1000, test_input_allocator);
    // defer test_input_allocator.free(boxes1000);

    // try bench("create index 1000 boxes gpa", createIndexFromBoxesAndFree, .{ boxes1000, gpa_allocator });
    // try bench("create index 1000 boxes arena", createIndexFromBoxesAndFree, .{ boxes1000, arena_allocator });
    // try bench("create index 1000 boxes c alloc", createIndexFromBoxesAndFree, .{ boxes1000, std.heap.c_allocator });

    // const points100000 = try createPointsOnCircle(100000, test_input_allocator);
    // defer test_input_allocator.free(points100000);
    // const boxes100000 = try createBoxesFromPointPairs(points100000, test_input_allocator);
    // defer test_input_allocator.free(boxes100000);

    // try bench("create index 100000 boxes gpa", createIndexFromBoxesAndFree, .{ boxes100000, gpa_allocator });
    // try bench("create index 100000 boxes arena", createIndexFromBoxesAndFree, .{ boxes100000, arena_allocator });
    // try bench("create index 100000 boxes c alloc", createIndexFromBoxesAndFree, .{ boxes100000, std.heap.c_allocator });

    const points1000000 = try createPointsOnCircle(1000000, test_input_allocator);
    defer test_input_allocator.free(points1000000);
    const boxes1000000 = try createBoxesFromPointPairs(points1000000, test_input_allocator);
    defer test_input_allocator.free(boxes1000000);

    try bench("create index 1000000 boxes gpa", createIndexFromBoxesAndFree, .{ boxes1000000, gpa_allocator });
    try bench("create index 1000000 boxes arena", createIndexFromBoxesAndFree, .{ boxes1000000, arena_allocator });
    try bench("create index 1000000 boxes c alloc", createIndexFromBoxesAndFree, .{ boxes1000000, std.heap.c_allocator });
}
