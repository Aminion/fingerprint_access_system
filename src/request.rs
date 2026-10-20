use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;

/// Where a task's worker posts the result of a command.
pub type Reply<T> = &'static Signal<CriticalSectionRawMutex, T>;

/// Send the command built by `cmd` (which carries `reply`) and wait for the result.
pub async fn request<C, T, const N: usize>(
    channel: &Channel<CriticalSectionRawMutex, C, N>,
    reply: Reply<T>,
    cmd: impl FnOnce(Reply<T>) -> C,
) -> T {
    channel.send(cmd(reply)).await;
    reply.wait().await
}
