from repulsor_3d_sim.config import load_config
from repulsor_3d_sim.app import ViewerApp

def main():
    cfg = load_config()
    app = ViewerApp(cfg)
    app.run()

if __name__ == "__main__":
    main()
