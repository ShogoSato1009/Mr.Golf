"""Export offscreen golf_demo frames to a constant-rate H.264 MP4."""

import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile


def export(demo: Path, config: Path, output: Path, ffmpeg: str) -> None:
    if output.suffix.lower() != ".mp4":
        raise ValueError("Output must have an .mp4 extension")
    if output.exists():
        raise FileExistsError(f"Output already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=output.parent) as directory:
        temporary = Path(directory) / "capture.mp4"
        # Keep the stream unbuffered: FFmpeg must receive all bytes after the header.
        with subprocess.Popen([demo.resolve(), "--frames", config.resolve()],
                              stdout=subprocess.PIPE, bufsize=0) as renderer:
            try:
                header = renderer.stdout.readline(64).split()
                if len(header) != 3:
                    raise RuntimeError("Renderer did not produce a frame header")
                width, height, fps = map(int, header)
                subprocess.run([
                    ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin", "-y",
                    "-f", "rawvideo", "-pixel_format", "bgr0",
                    "-video_size", f"{width}x{height}", "-framerate", str(fps), "-i", "pipe:0",
                    "-vf", "vflip", "-c:v", "libx264", "-preset", "fast", "-crf", "17",
                    "-pix_fmt", "yuv420p", "-movflags", "+faststart", temporary,
                ], stdin=renderer.stdout, check=True)
                if renderer.wait() != 0:
                    raise RuntimeError("Renderer failed while writing frames")
            except BaseException:
                renderer.kill()
                raise
        # On Windows rename refuses to overwrite an existing destination.
        temporary.rename(output.resolve())
    print(f"Saved {output.resolve()} ({width}x{height}, {fps} fps)")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", type=Path, required=True, help="path to golf_demo.exe")
    parser.add_argument("--config", type=Path, default=Path("configs/demo.cfg"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        parser.error("FFmpeg with libx264 must be on PATH")
    try:
        export(args.demo, args.config, args.output, ffmpeg)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        parser.exit(1, f"export_mp4: {error}\n")


if __name__ == "__main__":
    main()
