// Draws the launcher's icon from code (a flame in a rounded square), one PNG per
// iconset size. Build step only: fuelicon DIR.iconset, then iconutil makes the .icns.
#import <CoreGraphics/CoreGraphics.h>
#import <Foundation/Foundation.h>
#import <ImageIO/ImageIO.h>
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

static CGGradientRef gradient(CGColorSpaceRef cs, CGFloat r0, CGFloat g0, CGFloat b0, CGFloat r1, CGFloat g1, CGFloat b1) {
    const CGFloat c[8] = {r0, g0, b0, 1, r1, g1, b1, 1};
    return CGGradientCreateWithColorComponents(cs, c, nullptr, 2);
}

static void flame(CGContextRef g, CGFloat s, CGFloat cx, CGFloat cy, CGFloat scale) {
    // Unit teardrop (tip up) around (0.5, 0.5), scaled about its centre.
    auto P = [&](CGFloat x, CGFloat y) { return CGPointMake(cx + (x - 0.5) * scale * s, cy + (y - 0.5) * scale * s); };
    CGContextMoveToPoint(g, P(0.5, 0.86).x, P(0.5, 0.86).y);
    CGContextAddCurveToPoint(g, P(0.58, 0.68).x, P(0.58, 0.68).y, P(0.74, 0.55).x, P(0.74, 0.55).y, P(0.74, 0.38).x, P(0.74, 0.38).y);
    CGContextAddCurveToPoint(g, P(0.74, 0.24).x, P(0.74, 0.24).y, P(0.63, 0.14).x, P(0.63, 0.14).y, P(0.5, 0.14).x, P(0.5, 0.14).y);
    CGContextAddCurveToPoint(g, P(0.37, 0.14).x, P(0.37, 0.14).y, P(0.26, 0.24).x, P(0.26, 0.24).y, P(0.26, 0.38).x, P(0.26, 0.38).y);
    CGContextAddCurveToPoint(g, P(0.26, 0.55).x, P(0.26, 0.55).y, P(0.42, 0.68).x, P(0.42, 0.68).y, P(0.5, 0.86).x, P(0.5, 0.86).y);
    CGContextClosePath(g);
}

static bool render(int px, NSString* path) {
    CGColorSpaceRef cs = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
    CGContextRef g = CGBitmapContextCreate(nullptr, px, px, 8, 0, cs, kCGImageAlphaPremultipliedLast);
    const CGFloat s = px;
    // Rounded square with the macOS margin (about 10 %).
    const CGRect body = CGRectMake(s * 0.098, s * 0.098, s * 0.804, s * 0.804);
    CGPathRef shape = CGPathCreateWithRoundedRect(body, s * 0.18, s * 0.18, nullptr);
    CGContextSaveGState(g);
    CGContextAddPath(g, shape);
    CGContextClip(g);
    CGGradientRef bg = gradient(cs, 0.10, 0.11, 0.15, 0.27, 0.16, 0.13);
    CGContextDrawLinearGradient(g, bg, CGPointMake(0, s * 0.9), CGPointMake(0, s * 0.1), 0);
    // Road stripes sweeping up to a vanishing point.
    CGContextSetRGBFillColor(g, 1, 1, 1, 0.07);
    for (int i = -3; i <= 3; ++i) {
        CGContextMoveToPoint(g, s * 0.5 + i * s * 0.02, s * 0.5);
        CGContextAddLineToPoint(g, s * 0.5 + i * s * 0.16 + s * 0.04, s * 0.1);
        CGContextAddLineToPoint(g, s * 0.5 + i * s * 0.16 - s * 0.04, s * 0.1);
        CGContextClosePath(g);
        CGContextFillPath(g);
    }
    // Flame: orange body, yellow core.
    CGContextSaveGState(g);
    flame(g, s, s * 0.5, s * 0.52, 0.62);
    CGContextClip(g);
    CGGradientRef fire = gradient(cs, 1.0, 0.62, 0.12, 0.90, 0.22, 0.10);
    CGContextDrawLinearGradient(g, fire, CGPointMake(0, s * 0.82), CGPointMake(0, s * 0.2), 0);
    CGGradientRelease(fire);
    CGContextRestoreGState(g);
    CGContextSaveGState(g);
    flame(g, s, s * 0.5, s * 0.455, 0.30);
    CGContextClip(g);
    CGGradientRef core = gradient(cs, 1.0, 0.93, 0.55, 1.0, 0.72, 0.22);
    CGContextDrawLinearGradient(g, core, CGPointMake(0, s * 0.62), CGPointMake(0, s * 0.3), 0);
    CGGradientRelease(core);
    CGContextRestoreGState(g);
    CGContextRestoreGState(g);
    CGGradientRelease(bg);
    CGPathRelease(shape);

    CGImageRef img = CGBitmapContextCreateImage(g);
    NSURL* url = [NSURL fileURLWithPath:path];
    CGImageDestinationRef dst = CGImageDestinationCreateWithURL((__bridge CFURLRef)url, (__bridge CFStringRef)UTTypePNG.identifier, 1, nullptr);
    bool ok = dst != nullptr;
    if (ok) {
        CGImageDestinationAddImage(dst, img, nullptr);
        ok = CGImageDestinationFinalize(dst);
        CFRelease(dst);
    }
    CGImageRelease(img);
    CGContextRelease(g);
    CGColorSpaceRelease(cs);
    return ok;
}

int main(int argc, const char** argv) {
    @autoreleasepool {
        if (argc != 2) {
            fprintf(stderr, "usage: fuelicon OUT.iconset\n");
            return 2;
        }
        NSString* dir = [NSString stringWithUTF8String:argv[1]];
        [[NSFileManager defaultManager] createDirectoryAtPath:dir withIntermediateDirectories:YES attributes:nil error:nil];
        const struct { int pt, scale; } sizes[] = {{16, 1}, {16, 2}, {32, 1}, {32, 2}, {128, 1}, {128, 2}, {256, 1}, {256, 2}, {512, 1}, {512, 2}};
        for (const auto& z : sizes) {
            NSString* name = z.scale == 1 ? [NSString stringWithFormat:@"icon_%dx%d.png", z.pt, z.pt]
                                          : [NSString stringWithFormat:@"icon_%dx%d@2x.png", z.pt, z.pt];
            if (!render(z.pt * z.scale, [dir stringByAppendingPathComponent:name])) {
                fprintf(stderr, "fuelicon: cannot write %s\n", name.UTF8String);
                return 1;
            }
        }
    }
    return 0;
}
