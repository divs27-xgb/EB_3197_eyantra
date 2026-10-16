"""Read-only snapshot of CoppeliaSim contacts in the current dynamics pass."""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def main():
    client = RemoteAPIClient()
    sim = client.require("sim")
    print("Simulation time:", sim.getSimulationTime())
    print("Querying current contacts; no scene or motor commands are sent.")

    found = 0
    for index in range(100):
        try:
            colliding, point, force, normal = sim.getContactInfo(
                sim.handle_all, sim.handle_all, index
            )
        except Exception as exc:
            print(f"Contact list ended at index {index}: {type(exc).__name__}: {exc}")
            break

        if not colliding:
            print(f"No contact at index {index}; stopping contact scan.")
            break

        names = []
        for handle in colliding:
            try:
                names.append(sim.getObjectAlias(handle, 1))
            except Exception:
                names.append(f"handle {handle}")
        print(f"{index}: objects={names}; point={point}; force={force}; normal={normal}")
        found += 1

    if found == 0:
        print("No contacts returned in this snapshot (or contact data is unavailable at this instant).")


if __name__ == "__main__":
    main()
