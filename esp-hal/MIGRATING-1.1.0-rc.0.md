# Migration Guide from 1.0.0 to 1.1.0-rc.0

## I2S one-shot DMA uses buffers

I2S one-shot DMA APIs now use the DMA buffer types/traits instead of raw slices.

```diff
 let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
 let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
-i2s_tx.write_dma(&tx_buffer)?;
-i2s_rx.read_dma(&mut rx_buffer)?;
+let tx_transfer = i2s_tx.write_dma(dma_tx_buf).expect("start tx dma");
+let (_i2s_tx, _dma_tx_buf) = tx_transfer.wait().expect("finish tx dma");
+let rx_transfer = i2s_rx.read_dma(dma_rx_buf).expect("start rx dma");
+let (_i2s_rx, _dma_rx_buf) = rx_transfer.wait().expect("finish rx dma");
```

- `write_dma` / `read_dma` now use `DmaTxBuffer` / `DmaRxBuffer` inputs and
  return owning transfer handles.
- `read_dma` takes only the RX buffer; the byte count comes from [`DmaRxBuffer::peripheral_rx_dma_length`] (for [`DmaRxBuf`], that is [`DmaRxBuf::len`]). Streaming RX uses the same `read_dma` with a ring buffer type.
- async one-shot APIs now take `&mut DmaTxBuf` / `&mut DmaRxBuf`.

## I2S streaming DMA merged into `write_dma` / `read_dma`

`DmaTxCircularBuffer` / `DmaRxCircularBuffer` are removed. Ring buffers implement
`DmaTxBuffer` / `DmaRxBuffer` and report streaming via `tx_stream_state` /
`rx_stream_state`. Use `i2s_tx.write_dma(tx_circular_buf)` and
`i2s_rx.read_dma(rx_circular_buf)` instead of `write_dma_circular` /
`read_dma_circular`. Transfer handles expose `push` / `pop` / `stop` only when
the buffer was streaming (`WrongTransferMode` if you call `wait` on those).
