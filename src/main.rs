mod car;
mod game;
mod physics;

use game::Game;
use kajiya_simple::*;
use structopt::StructOpt;

#[derive(Debug, StructOpt)]
#[structopt(name = "cornell-mcray", about = "Cornell McRay t'Racing")]
struct Opt {
    #[structopt(long, default_value = "1920")]
    width: u32,

    #[structopt(long, default_value = "1080")]
    height: u32,

    #[structopt(long, default_value = "1.0")]
    temporal_upsampling: f32,

    #[structopt(long)]
    no_vsync: bool,

    #[structopt(long)]
    fullscreen: bool,
}

fn main() -> anyhow::Result<()> {
    let opt = Opt::from_args();

    // Point `kajiya` to standard assets and shaders
    set_standard_vfs_mount_points("../kajiya");

    // Game-specific assets
    set_vfs_mount_point("/baked", "./baked");

    let mut kajiya = SimpleMainLoop::builder()
        .vsync(!opt.no_vsync)
        .temporal_upsampling(opt.temporal_upsampling)
        .resolution([opt.width, opt.height])
        .fullscreen(opt.fullscreen.then(|| FullscreenMode::Exclusive))
        .build(
            WindowBuilder::new()
                .with_title("Cornell McRay t'Racer")
                .with_resizable(false),
        )?;

    // Fit the CSGI volume to the game world
    kajiya.world_renderer.world_gi_scale = 7.0;

    let mut game = Game::new(&mut kajiya.world_renderer)?;
    kajiya.run(move |ctx| game.frame(ctx))
}
