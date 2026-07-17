# Security and privacy

Do not commit raw device captures, serial numbers, terminal logs, credentials, private
keys, machine environment files, or build/install trees. Store local evidence under
`artifacts/`; the directory is ignored by Git.

The setup script grants Linux capabilities only to the project-owned EtherCAT
executables. Re-run it after relinking those executables, and do not grant capabilities
to the system-wide `ros2` command.

Before sharing the repository, review `git status`, `git diff --cached`, and the
repository visibility on GitHub.
