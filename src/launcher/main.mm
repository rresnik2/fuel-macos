// FUEL.app launcher: picks the game folder, edits the shared settings file
// (core/settings.hpp) and starts the port's own game binary, fuelview, which sits
// next to this executable in Contents/MacOS. No game data lives in the app.
//
// Test switches (not needed by players): --play starts the game as soon as the window
// is up; --log FILE writes the launcher's own events (arguments, exit codes) to FILE.
#import <Cocoa/Cocoa.h>

#include "core/settings.hpp"

#include <algorithm>
#include <filesystem>

namespace fs = std::filesystem;

namespace {

NSString* gLogFile = nil;  // --log FILE

void logEvent(NSString* line) {
    NSLog(@"%@", line);
    if (!gLogFile) return;
    NSFileHandle* h = [NSFileHandle fileHandleForWritingAtPath:gLogFile];
    if (!h) {
        [[NSFileManager defaultManager] createFileAtPath:gLogFile contents:nil attributes:nil];
        h = [NSFileHandle fileHandleForWritingAtPath:gLogFile];
    }
    [h seekToEndOfFile];
    [h writeData:[[line stringByAppendingString:@"\n"] dataUsingEncoding:NSUTF8StringEncoding]];
    [h closeFile];
}

NSString* ns(const std::string& s) { return [NSString stringWithUTF8String:s.c_str()]; }

// What a game folder must hold; the port reads FUEL.exe for its engine tables.
struct Required {
    const char* path;
    bool directory;
};
const Required kRequired[] = {
    {"CommonComponent/WORLD/USA1.DPC", false},
    {"LocalizedComponent/GameTsc", true},
    {"XFinalComponent/FUEL.exe", false},
};

// nil when `dir` is a usable game folder, else a sentence naming the first missing item.
NSString* validateGameFolder(const std::string& dir) {
    if (dir.empty()) return @"No game folder chosen yet.";
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) return [NSString stringWithFormat:@"The folder %@ does not exist.", ns(dir)];
    for (const Required& r : kRequired) {
        const fs::path p = fs::path(dir) / r.path;
        const bool ok = r.directory ? fs::is_directory(p, ec) : fs::is_regular_file(p, ec);
        if (!ok) return [NSString stringWithFormat:@"Missing %s%s in the folder.", r.path, r.directory ? "/" : ""];
    }
    return nil;
}

struct Resolution {
    int w, h;
};

}  // namespace

@interface Launcher : NSObject <NSApplicationDelegate>
@end

@implementation Launcher {
    NSWindow* _window;
    NSStackView* _stack;
    NSTextField* _folderPath;
    NSTextField* _folderStatus;
    NSPopUpButton* _resolution;
    NSPopUpButton* _display;
    NSButton* _post;
    NSButton* _grass;
    NSSlider* _volume;
    NSTextField* _volumeLabel;
    NSPopUpButton* _language;
    NSButton* _play;
    NSStackView* _logBox;
    NSTextView* _logView;
    fuel::Settings _s;
    NSTask* _task;
    BOOL _autoPlay;
    BOOL _building;  // controls are being filled in: do not save
}

- (instancetype)initWithAutoPlay:(BOOL)autoPlay {
    if ((self = [super init])) _autoPlay = autoPlay;
    return self;
}

// --- UI construction -------------------------------------------------------------------

static NSTextField* label(NSString* text, NSFont* font = nil, NSColor* color = nil) {
    NSTextField* t = [NSTextField labelWithString:text];
    if (font) t.font = font;
    if (color) t.textColor = color;
    return t;
}

- (NSPopUpButton*)popup {
    NSPopUpButton* p = [[NSPopUpButton alloc] initWithFrame:NSZeroRect pullsDown:NO];
    p.target = self;
    p.action = @selector(controlChanged:);
    return p;
}

- (NSButton*)check:(NSString*)title {
    NSButton* b = [NSButton checkboxWithTitle:title target:self action:@selector(controlChanged:)];
    return b;
}

// The display's native pixel size.
- (Resolution)nativeResolution {
    NSScreen* sc = NSScreen.mainScreen ?: NSScreen.screens.firstObject;
    const NSSize px = [sc convertRectToBacking:sc.frame].size;
    return {int(px.width), int(px.height)};
}

- (void)fillResolutions {
    [_resolution removeAllItems];
    const Resolution native = [self nativeResolution];
    const Resolution presets[] = {{1280, 720}, {1280, 800}, {1440, 900}, {1600, 900}, {1600, 1000}, {1920, 1080}, {1920, 1200}, {2560, 1440}, {2560, 1600}};
    std::vector<Resolution> list;
    for (const Resolution& r : presets)
        if (r.w < native.w || (r.w == native.w && r.h < native.h)) list.push_back(r);
    list.push_back(native);
    BOOL found = NO;
    for (const Resolution& r : list) {
        const BOOL isNative = r.w == native.w && r.h == native.h;
        [_resolution addItemWithTitle:[NSString stringWithFormat:@"%d x %d%@", r.w, r.h, isNative ? @" (display's native size)" : @""]];
        _resolution.lastItem.representedObject = [NSValue valueWithSize:NSMakeSize(r.w, r.h)];
        if (r.w == _s.width && r.h == _s.height) found = YES;
    }
    if (!found) {  // a size typed into settings.txt by hand
        [_resolution addItemWithTitle:[NSString stringWithFormat:@"%d x %d (custom)", _s.width, _s.height]];
        _resolution.lastItem.representedObject = [NSValue valueWithSize:NSMakeSize(_s.width, _s.height)];
    }
    for (NSMenuItem* it in _resolution.itemArray) {
        const NSSize z = [it.representedObject sizeValue];
        if (int(z.width) == _s.width && int(z.height) == _s.height) [_resolution selectItem:it];
    }
}

- (void)buildWindow {
    _window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 540, 400)
                                          styleMask:NSWindowStyleMaskTitled | NSWindowStyleMaskClosable | NSWindowStyleMaskMiniaturizable
                                            backing:NSBackingStoreBuffered
                                              defer:NO];
    _window.title = @"FUEL";
    _window.releasedWhenClosed = NO;

    NSTextField* title = label(@"FUEL", [NSFont systemFontOfSize:30 weight:NSFontWeightHeavy]);
    NSTextField* sub = label(@"macOS port: launcher", [NSFont systemFontOfSize:12], NSColor.secondaryLabelColor);

    // Game folder
    NSTextField* folderCaption = label(@"Game folder", [NSFont systemFontOfSize:12 weight:NSFontWeightSemibold]);
    _folderPath = label(@"", [NSFont systemFontOfSize:12]);
    _folderPath.lineBreakMode = NSLineBreakByTruncatingMiddle;
    _folderPath.selectable = YES;
    [_folderPath setContentCompressionResistancePriority:250 forOrientation:NSLayoutConstraintOrientationHorizontal];
    NSButton* choose = [NSButton buttonWithTitle:@"Choose…" target:self action:@selector(chooseFolder:)];
    NSStackView* folderRow = [NSStackView stackViewWithViews:@[_folderPath, choose]];
    folderRow.spacing = 8;
    _folderStatus = label(@"", [NSFont systemFontOfSize:12]);
    _folderStatus.maximumNumberOfLines = 3;
    _folderStatus.preferredMaxLayoutWidth = 500;

    // Settings
    _resolution = [self popup];
    _display = [self popup];
    [_display addItemsWithTitles:@[@"Windowed", @"Fullscreen"]];
    _post = [self check:@"Post effects (bloom, colour grade, motion blur)"];
    _grass = [self check:@"Grass"];
    _volume = [NSSlider sliderWithValue:1 minValue:0 maxValue:1 target:self action:@selector(controlChanged:)];
    _volume.continuous = YES;
    [_volume.widthAnchor constraintEqualToConstant:200].active = YES;
    _volumeLabel = label(@"100%", [NSFont monospacedDigitSystemFontOfSize:12 weight:NSFontWeightRegular]);
    NSStackView* volumeRow = [NSStackView stackViewWithViews:@[_volume, _volumeLabel]];
    volumeRow.spacing = 8;
    _language = [self popup];
    const struct { const char* code; const char* name; } langs[] = {
        {"en", "English"}, {"fr", "Français"}, {"de", "Deutsch"}, {"es", "Español"}, {"it", "Italiano"}};
    for (const auto& l : langs) {
        [_language addItemWithTitle:ns(l.name)];
        _language.lastItem.representedObject = ns(l.code);
    }
    NSGridView* grid = [NSGridView gridViewWithViews:@[
        @[label(@"Resolution"), _resolution],
        @[label(@"Display"), _display],
        @[label(@"Graphics"), _post],
        @[[NSGridCell emptyContentView], _grass],
        @[label(@"Volume"), volumeRow],
        @[label(@"Language"), _language],
    ]];
    grid.rowSpacing = 8;
    grid.columnSpacing = 12;
    [grid columnAtIndex:0].xPlacement = NSGridCellPlacementTrailing;

    // Buttons
    NSButton* quit = [NSButton buttonWithTitle:@"Quit" target:NSApp action:@selector(terminate:)];
    _play = [NSButton buttonWithTitle:@"Play" target:self action:@selector(play:)];
    _play.keyEquivalent = @"\r";
    _play.bezelStyle = NSBezelStyleRounded;
    NSView* spacer = [NSView new];
    [spacer setContentHuggingPriority:1 forOrientation:NSLayoutConstraintOrientationHorizontal];
    NSStackView* buttons = [NSStackView stackViewWithViews:@[quit, spacer, _play]];
    buttons.spacing = 8;

    // Last run log (shown only after an error)
    NSTextField* logCaption = label(@"Last run log", [NSFont systemFontOfSize:12 weight:NSFontWeightSemibold]);
    NSScrollView* scroll = [[NSScrollView alloc] initWithFrame:NSMakeRect(0, 0, 500, 160)];
    scroll.hasVerticalScroller = YES;
    scroll.borderType = NSBezelBorder;
    _logView = [[NSTextView alloc] initWithFrame:scroll.bounds];
    _logView.editable = NO;
    _logView.font = [NSFont monospacedSystemFontOfSize:11 weight:NSFontWeightRegular];
    _logView.autoresizingMask = NSViewWidthSizable;
    scroll.documentView = _logView;
    [scroll.heightAnchor constraintEqualToConstant:160].active = YES;
    _logBox = [NSStackView stackViewWithViews:@[logCaption, scroll]];
    _logBox.orientation = NSUserInterfaceLayoutOrientationVertical;
    _logBox.alignment = NSLayoutAttributeLeading;
    _logBox.spacing = 4;
    _logBox.hidden = YES;
    [scroll.widthAnchor constraintEqualToAnchor:_logBox.widthAnchor].active = YES;

    NSBox* rule1 = [NSBox new];
    rule1.boxType = NSBoxSeparator;
    NSBox* rule2 = [NSBox new];
    rule2.boxType = NSBoxSeparator;

    _stack = [NSStackView stackViewWithViews:@[title, sub, folderCaption, folderRow, _folderStatus, rule1, grid, rule2, buttons, _logBox]];
    _stack.orientation = NSUserInterfaceLayoutOrientationVertical;
    _stack.alignment = NSLayoutAttributeLeading;
    _stack.spacing = 10;
    [_stack setCustomSpacing:2 afterView:title];
    [_stack setCustomSpacing:18 afterView:sub];
    _stack.edgeInsets = NSEdgeInsetsMake(22, 20, 20, 20);
    _stack.detachesHiddenViews = YES;
    for (NSView* v in @[folderRow, rule1, rule2, buttons, _logBox])
        [v.widthAnchor constraintEqualToAnchor:_stack.widthAnchor constant:-40].active = YES;
    [_stack.widthAnchor constraintEqualToConstant:540].active = YES;
    _window.contentView = _stack;
    [self fitWindow];
}

- (void)fitWindow {
    [_stack layoutSubtreeIfNeeded];
    [_window setContentSize:_stack.fittingSize];
}

// --- Settings <-> controls -----------------------------------------------------------------

- (void)loadControls {
    _building = YES;
    _s = fuel::loadSettings();
    [self fillResolutions];
    [_display selectItemAtIndex:_s.fullscreen ? 1 : 0];
    _post.state = _s.postEffects ? NSControlStateValueOn : NSControlStateValueOff;
    _grass.state = _s.grass ? NSControlStateValueOn : NSControlStateValueOff;
    _volume.doubleValue = std::clamp(_s.volume, 0.0f, 1.0f);
    _volumeLabel.stringValue = [NSString stringWithFormat:@"%d%%", int(_volume.doubleValue * 100 + 0.5)];
    for (NSMenuItem* it in _language.itemArray)
        if ([it.representedObject isEqualToString:ns(_s.language)]) [_language selectItem:it];
    _building = NO;
}

- (void)controlChanged:(id)sender {
    if (_building) return;
    const NSSize z = [_resolution.selectedItem.representedObject sizeValue];
    _s.width = int(z.width);
    _s.height = int(z.height);
    _s.fullscreen = _display.indexOfSelectedItem == 1;
    _s.postEffects = _post.state == NSControlStateValueOn;
    _s.grass = _grass.state == NSControlStateValueOn;
    _s.volume = float(int(_volume.doubleValue * 100 + 0.5)) / 100.0f;
    _volumeLabel.stringValue = [NSString stringWithFormat:@"%d%%", int(_s.volume * 100 + 0.5)];
    if (NSString* code = _language.selectedItem.representedObject) _s.language = code.UTF8String;
    [self save];
}

- (void)save {
    if (!fuel::saveSettings(_s)) logEvent(@"launcher: cannot write the settings file");
}

// --- Game folder --------------------------------------------------------------------------

- (void)showFolder {
    _folderPath.stringValue = _s.dataDir.empty() ? @"(none)" : ns(_s.dataDir);
    NSString* problem = validateGameFolder(_s.dataDir);
    if (problem) {
        _folderStatus.stringValue = problem;
        _folderStatus.textColor = _s.dataDir.empty() ? NSColor.secondaryLabelColor : NSColor.systemRedColor;
    } else {
        _folderStatus.stringValue = @"Found: world data, text tables and FUEL.exe.";
        _folderStatus.textColor = NSColor.systemGreenColor;
    }
    _play.enabled = problem == nil && _task == nil;
    [self fitWindow];
}

- (void)chooseFolder:(id)sender {
    NSOpenPanel* panel = [NSOpenPanel openPanel];
    panel.canChooseFiles = NO;
    panel.canChooseDirectories = YES;
    panel.allowsMultipleSelection = NO;
    panel.prompt = @"Use This Folder";
    panel.message = @"Choose the folder of your FUEL install: the one that holds CommonComponent, LocalizedComponent and XFinalComponent.";
    if (!_s.dataDir.empty()) panel.directoryURL = [NSURL fileURLWithPath:ns(_s.dataDir)];
    [panel beginSheetModalForWindow:_window completionHandler:^(NSModalResponse r) {
        if (r != NSModalResponseOK) {
            [self showFolder];
            return;
        }
        const std::string dir = panel.URL.path.UTF8String;
        NSString* problem = validateGameFolder(dir);
        if (problem) {
            NSAlert* a = [NSAlert new];
            a.messageText = @"That is not a FUEL game folder";
            a.informativeText = [NSString stringWithFormat:@"%@\n\nChoose the folder that holds CommonComponent/, LocalizedComponent/ and XFinalComponent/FUEL.exe, as extracted from your own copy of the game.", problem];
            a.alertStyle = NSAlertStyleWarning;
            [a addButtonWithTitle:@"Choose Again"];
            [a addButtonWithTitle:@"Cancel"];
            [a beginSheetModalForWindow:self->_window completionHandler:^(NSModalResponse ar) {
                if (ar == NSAlertFirstButtonReturn) [self chooseFolder:nil];
                else [self showFolder];
            }];
            return;
        }
        self->_s.dataDir = dir;
        [self save];
        [self showFolder];
    }];
}

// --- Play ------------------------------------------------------------------------------------

- (NSURL*)gameBinary {
    return [[NSBundle.mainBundle.executableURL URLByDeletingLastPathComponent] URLByAppendingPathComponent:@"fuelview"];
}

- (void)setLog:(NSString*)text {
    _logView.string = text;
    _logBox.hidden = text.length == 0;
    [self fitWindow];
}

- (void)play:(id)sender {
    if (_task) return;
    [self controlChanged:nil];  // the controls are the truth
    NSString* problem = validateGameFolder(_s.dataDir);
    if (problem) {
        [self showFolder];
        return;
    }
    [self save];
    [self setLog:@""];
    // fuelview reads the settings file itself.
    [self startGame:@[@"--game", @"--data", ns(_s.dataDir)] fallback:YES];
}

// Starts fuelview with `args`. When `fallback` is set and this fuelview does not know --game yet,
// starts the fallback run instead.
- (void)startGame:(NSArray<NSString*>*)args fallback:(BOOL)allowFallback {
    NSURL* exe = [self gameBinary];
    if (![NSFileManager.defaultManager isExecutableFileAtPath:exe.path]) {
        [self setLog:[NSString stringWithFormat:@"The game binary is missing: %@", exe.path]];
        return;
    }
    NSTask* task = [NSTask new];
    task.executableURL = exe;
    task.arguments = args;
    NSPipe* err = [NSPipe pipe];
    task.standardError = err;
    task.standardOutput = NSFileHandle.fileHandleWithNullDevice;
    NSMutableData* captured = [NSMutableData data];
    NSFileHandle* reader = err.fileHandleForReading;
    reader.readabilityHandler = ^(NSFileHandle* h) {
        NSData* d = h.availableData;
        if (d.length == 0) return;
        @synchronized(captured) { [captured appendData:d]; }
        fwrite(d.bytes, 1, d.length, stderr);
    };
    NSString* command = [NSString stringWithFormat:@"%@ %@", exe.path, [args componentsJoinedByString:@" "]];
    logEvent([@"launcher: starting " stringByAppendingString:command]);
    const NSTimeInterval started = NSDate.timeIntervalSinceReferenceDate;
    task.terminationHandler = ^(NSTask* t) {
        reader.readabilityHandler = nil;
        NSData* rest = [reader readDataToEndOfFile];
        if (rest.length) @synchronized(captured) { [captured appendData:rest]; }
        dispatch_async(dispatch_get_main_queue(), ^{
            NSString* text;
            @synchronized(captured) { text = [[NSString alloc] initWithData:captured encoding:NSUTF8StringEncoding] ?: @""; }
            [self gameEnded:t status:t.terminationStatus reason:t.terminationReason log:text command:command
                    seconds:NSDate.timeIntervalSinceReferenceDate - started fallback:allowFallback];
        });
    };
    NSError* e = nil;
    if (![task launchAndReturnError:&e]) {
        [self setLog:[NSString stringWithFormat:@"Could not start %@\n%@", exe.path, e.localizedDescription]];
        return;
    }
    _task = task;
    _play.enabled = NO;
    _play.title = @"Running…";
}

- (void)gameEnded:(NSTask*)task status:(int)status reason:(NSTaskTerminationReason)reason log:(NSString*)log command:(NSString*)command
          seconds:(NSTimeInterval)seconds fallback:(BOOL)allowFallback {
    logEvent([NSString stringWithFormat:@"launcher: game exited, status %d after %.1f s", status, seconds]);
    _task = nil;
    _play.title = @"Play";

    // FALLBACK (remove once P19's `fuelview --game` is on main): a fuelview without --game stops at once
    // with "unknown option --game". Then start the old driving demo, which reads the same settings file
    // through --settings, so Play still goes somewhere.
    if (allowFallback && status != 0 && seconds < 15 && [log containsString:@"unknown option --game"]) {
        logEvent(@"launcher: this fuelview has no --game yet; starting the fallback driving demo");
        [self startGame:@[@"--data", ns(_s.dataDir), @"--assemble", @"QUAD_ROAD_2", @"--drive", @"--at", @"-39700,-28300", @"--settings"]
               fallback:NO];
        if (_task) return;
    }

    [self showFolder];  // re-enables Play
    if (status != 0) {
        NSArray<NSString*>* lines = [log componentsSeparatedByString:@"\n"];
        if (lines.count > 200) lines = [lines subarrayWithRange:NSMakeRange(lines.count - 200, 200)];
        NSString* how = reason == NSTaskTerminationReasonUncaughtSignal ? [NSString stringWithFormat:@"crashed (signal %d)", status]
                                                                        : [NSString stringWithFormat:@"exited with status %d", status];
        [self setLog:[NSString stringWithFormat:@"$ %@\nThe game %@.\n\n%@", command, how, [lines componentsJoinedByString:@"\n"]]];
    }
}

// --- App --------------------------------------------------------------------------------------

- (void)applicationDidFinishLaunching:(NSNotification*)n {
    [self buildWindow];
    [self loadControls];
    [self showFolder];
    [_window center];
    [_window makeKeyAndOrderFront:nil];
    [NSApp activateIgnoringOtherApps:YES];
    if (_s.dataDir.empty()) {  // first run
        dispatch_async(dispatch_get_main_queue(), ^{ [self chooseFolder:nil]; });
    } else if (_autoPlay && _play.enabled) {
        dispatch_async(dispatch_get_main_queue(), ^{ [self play:nil]; });
    }
    // Writes the file on first run so the game and the launcher always agree on its contents.
    [self save];
}

- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication*)app { return YES; }

@end

int main(int argc, const char** argv) {
    @autoreleasepool {
        BOOL autoPlay = NO;
        for (int i = 1; i < argc; ++i) {
            if (!strcmp(argv[i], "--play")) autoPlay = YES;
            else if (!strcmp(argv[i], "--log") && i + 1 < argc) gLogFile = [NSString stringWithUTF8String:argv[++i]];
        }
        [NSApplication sharedApplication];
        [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];
        Launcher* app = [[Launcher alloc] initWithAutoPlay:autoPlay];
        NSApp.delegate = app;

        NSMenu* bar = [NSMenu new];
        NSMenuItem* appItem = [NSMenuItem new];
        [bar addItem:appItem];
        NSMenu* appMenu = [NSMenu new];
        [appMenu addItemWithTitle:@"Quit FUEL" action:@selector(terminate:) keyEquivalent:@"q"];
        appItem.submenu = appMenu;
        NSApp.mainMenu = bar;

        [NSApp run];
    }
    return 0;
}
